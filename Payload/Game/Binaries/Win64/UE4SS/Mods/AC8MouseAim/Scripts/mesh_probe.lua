-- One-shot, read-only runtime geometry probe: writes a JSON snapshot beside the script.
-- It does not inspect game archives or retain live UObject handles. Taken automatically
-- while telemetry is on (a recording session): once for each aircraft the player flies,
-- 5 s after it appears, and once when each boss (LADON, Moon 11) first appears. (No key: F11 is the game's
-- fullscreen toggle.)
local M = { requested = false }
local AUTO_DELAY, BOSS_CHECK = 5, 5
local seen_class, seen_since, captured, next_boss_check = nil, 0, {}, 0
local BOSSES = { 'BP_OP0045_ladn_CP_C', 'BP_OP1016_x40a_CP_Tewy_C' }
local boss_captured = {}
local function new_array() return setmetatable({}, { __json_array = true }) end

function M.request() M.requested = true end

local function get(obj, key)
    if not obj then return nil end
    local ok, value = pcall(function() return obj[key] end)
    return ok and value or nil
end

local function call(obj, name, ...)
    if not obj then return nil end
    local ok, value = pcall(function(...) return obj[name](obj, ...) end, ...)
    return ok and value or nil
end

local function valid(obj)
    local ok, result = pcall(function() return obj and obj:IsValid() end)
    return ok and result == true
end

local function full_name(obj)
    if not valid(obj) then return nil end
    return call(obj, 'GetFullName')
end

local function number(value)
    if type(value) == 'number' then return value end
    local ok, result = pcall(function() return value:get() end)
    return ok and type(result) == 'number' and result or nil
end

local function xyz(value)
    if not value then return nil end
    local x, y, z = number(get(value, 'X')), number(get(value, 'Y')), number(get(value, 'Z'))
    if x and y and z then return { x, y, z } end
    return nil
end

local function rotation(value)
    if not value then return nil end
    local p, y, r = number(get(value, 'Pitch')), number(get(value, 'Yaw')), number(get(value, 'Roll'))
    if p and y and r then return { p, y, r } end
    local x, yy, z, w = number(get(value, 'X')), number(get(value, 'Y')),
        number(get(value, 'Z')), number(get(value, 'W'))
    if x and yy and z and w then return { x, yy, z, w } end
    return nil
end

local function transform(value)
    if not value then return nil end
    return {
        translation = xyz(get(value, 'Translation')) or xyz(get(value, 'Location')),
        rotation = rotation(get(value, 'Rotation')),
        scale = xyz(get(value, 'Scale3D')),
    }
end

local function each(array, limit, visit)
    if not array then return 0, 'absent' end
    local count = 0
    local function unpack(value)
        local ok, result = pcall(function() return value:get() end)
        return ok and result ~= nil and result or value
    end
    local ok, err = pcall(function()
        if type(array) == 'table' then
            for _, value in ipairs(array) do
                count = count + 1
                if count > limit then break end
                visit(unpack(value))
            end
        else
            array:ForEach(function(_, element)
                count = count + 1
                if count > limit then return true end
                visit(unpack(element))
            end)
        end
    end)
    return count, ok and nil or tostring(err)
end

local function numbers(array, limit)
    local out = new_array()
    local count, err = each(array, limit, function(value)
        local n = number(value)
        if n then out[#out + 1] = n end
    end)
    return out, count, err
end

local function vertices(array, limit)
    local out = new_array()
    local count, err = each(array, limit, function(value)
        local v = xyz(value)
        if v then out[#out + 1] = v end
    end)
    return out, count, err
end

local function shape_list(geom, field, limit, describe)
    local result = { items = new_array() }
    result.count, result.error = each(get(geom, field), limit, function(value)
        result.items[#result.items + 1] = describe(value)
    end)
    result.truncated = result.count > limit
    return result
end

local function body_setup(body, component)
    if not valid(body) then return nil end
    local result = { name = full_name(body) }
    local bone = get(body, 'BoneName')
    result.bone = bone and call(bone, 'ToString') or nil
    if bone and result.bone then
        -- RTS_World is 0. A missing binding simply leaves this field absent.
        result.bone_world = transform(call(component, 'GetBoneTransform', bone, 0))
    end
    local geom = get(body, 'AggGeom')
    if not geom then result.error = 'AggGeom unavailable'; return result end
    result.convex = shape_list(geom, 'ConvexElems', 128, function(shape)
        local points, point_count, point_error = vertices(get(shape, 'VertexData'), 2048)
        local indices, index_count, index_error = numbers(get(shape, 'IndexData'), 8192)
        return {
            vertices = points, vertex_count = point_count, vertex_error = point_error,
            indices = indices, index_count = index_count, index_error = index_error,
            transform = transform(get(shape, 'Transform')),
        }
    end)
    result.boxes = shape_list(geom, 'BoxElems', 128, function(shape)
        return { center = xyz(get(shape, 'Center')), rotation = rotation(get(shape, 'Rotation')),
            x = number(get(shape, 'X')), y = number(get(shape, 'Y')), z = number(get(shape, 'Z')) }
    end)
    result.spheres = shape_list(geom, 'SphereElems', 128, function(shape)
        return { center = xyz(get(shape, 'Center')), radius = number(get(shape, 'Radius')) }
    end)
    result.capsules = shape_list(geom, 'SphylElems', 128, function(shape)
        return { center = xyz(get(shape, 'Center')), rotation = rotation(get(shape, 'Rotation')),
            radius = number(get(shape, 'Radius')), length = number(get(shape, 'Length')) }
    end)
    return result
end

local function component_snapshot(component)
    local item = { name = full_name(component), class = full_name(call(component, 'GetClass')) }
    item.parent = full_name(call(component, 'GetAttachParent'))
    item.relative = transform(call(component, 'GetRelativeTransform')) or {
        translation = xyz(get(component, 'RelativeLocation')),
        rotation = rotation(get(component, 'RelativeRotation')),
        scale = xyz(get(component, 'RelativeScale3D')),
    }
    item.world = transform(call(component, 'K2_GetComponentToWorld'))
    if not item.world or not item.world.translation then
        item.world = { translation = xyz(call(component, 'K2_GetComponentLocation')),
            rotation = rotation(call(component, 'K2_GetComponentRotation')),
            scale = xyz(call(component, 'K2_GetComponentScale')) }
    end
    local mesh = call(component, 'GetSkeletalMeshAsset') or call(component, 'GetStaticMesh')
        or get(component, 'SkeletalMesh') or get(component, 'SkeletalMeshAsset')
        or get(component, 'SkinnedAsset') or get(component, 'StaticMesh')
    item.mesh = full_name(mesh)
    item.lod_count = number(call(component, 'GetNumLODs'))
    item.predicted_lod = number(call(component, 'GetPredictedLODLevel'))
    local bounds = get(component, 'Bounds')
    if bounds then
        item.bounds = { origin = xyz(get(bounds, 'Origin')),
            extent = xyz(get(bounds, 'BoxExtent')),
            radius = number(get(bounds, 'SphereRadius')) }
    end
    local asset_bounds = call(mesh, 'GetBounds')
    if asset_bounds then
        item.asset_bounds = { origin = xyz(get(asset_bounds, 'Origin')),
            extent = xyz(get(asset_bounds, 'BoxExtent')),
            radius = number(get(asset_bounds, 'SphereRadius')) }
    end
    local bone_count = number(call(component, 'GetNumBones'))
    if bone_count and bone_count > 0 then
        item.bone_count = bone_count
        item.bones = new_array()
        for i = 0, math.min(bone_count, 256) - 1 do
            local name = call(component, 'GetBoneName', i)
            local label = name and call(name, 'ToString')
            if label then
                item.bones[#item.bones + 1] = { name = label,
                    component = transform(call(component, 'GetBoneTransform', name, 2)) }
            end
        end
        item.bones_truncated = bone_count > 256
    end
    local physics = call(component, 'GetPhysicsAsset') or get(component, 'PhysicsAsset')
        or (valid(mesh) and get(mesh, 'PhysicsAsset'))
    item.physics_asset = full_name(physics)
    local body = call(component, 'GetBodySetup') or get(component, 'BodySetup')
        or call(mesh, 'GetBodySetup') or get(mesh, 'BodySetup')
    item.body = body_setup(body, component)
    if valid(physics) then
        item.physics_bodies = { items = new_array() }
        local bodies = get(physics, 'SkeletalBodySetups')
        item.physics_bodies.count, item.physics_bodies.error = each(bodies, 128, function(value)
            item.physics_bodies.items[#item.physics_bodies.items + 1] = body_setup(value, component)
        end)
        item.physics_bodies.truncated = item.physics_bodies.count > 128
    end
    return item
end

local function json(value)
    local kind = type(value)
    if kind == 'nil' then return 'null' end
    if kind == 'boolean' then return tostring(value) end
    if kind == 'number' then
        return value == value and value ~= math.huge and value ~= -math.huge and tostring(value) or 'null'
    end
    if kind == 'string' then
        return '"' .. value:gsub('[%z\1-\31\\"]', function(c)
            local escapes = { ['\\'] = '\\\\', ['"'] = '\\"', ['\n'] = '\\n', ['\r'] = '\\r', ['\t'] = '\\t' }
            return escapes[c] or string.format('\\u%04x', c:byte())
        end) .. '"'
    end
    if kind ~= 'table' then return 'null' end
    if #value > 0 or (getmetatable(value) and getmetatable(value).__json_array) then
        local parts = {}
        for i = 1, #value do parts[i] = json(value[i]) end
        return '[' .. table.concat(parts, ',') .. ']'
    end
    local parts = {}
    for key, item in pairs(value) do parts[#parts + 1] = json(tostring(key)) .. ':' .. json(item) end
    return '{' .. table.concat(parts, ',') .. '}'
end

local function actor_snapshot(actor, actor_component)
    local result = { actor = full_name(actor),
        class = full_name(call(actor, 'GetClass')),
        actor_world = { translation = xyz(call(actor, 'K2_GetActorLocation')),
            rotation = rotation(call(actor, 'K2_GetActorRotation')) },
        components = new_array() }
    local components = call(actor, 'K2_GetComponentsByClass', actor_component)
    if not components then result.component_error = 'K2_GetComponentsByClass unavailable'; return result end
    result.component_count, result.component_error = each(components, 256, function(component)
        if valid(component) then
            local item_ok, item = pcall(component_snapshot, component)
            result.components[#result.components + 1] = item_ok and item
                or { name = full_name(component), error = tostring(item) }
        end
    end)
    result.truncated = result.component_count > 256
    return result
end

-- Decide whether to take a snapshot this frame (auto: telemetry is on).
local function due(pawn, auto)
    if M.requested then return true end
    if not auto then return false end
    local now = os.clock()
    local cls = call(call(call(pawn, 'GetClass'), 'GetFName'), 'ToString')
    if cls and cls ~= seen_class then seen_class, seen_since = cls, now end
    if cls and not captured[cls] and now - seen_since >= AUTO_DELAY then captured[cls] = true; return true end
    if now >= next_boss_check then
        next_boss_check = now + BOSS_CHECK
        for _, name in ipairs(BOSSES) do
            if not boss_captured[name] then
                local ok, boss = pcall(FindFirstOf, name)
                if ok and valid(boss) then boss_captured[name] = true; return true end
            end
        end
    end
    return false
end

function M.update(pawn, directory, auto)
    if not due(pawn, auto) then return end
    M.requested = false
    local ok, err = pcall(function()
        local actor_component = StaticFindObject('/Script/Engine.ActorComponent')
        assert(valid(actor_component), 'ActorComponent class unavailable')
        local snapshot = actor_snapshot(pawn, actor_component)
        snapshot.version = 2
        snapshot.captured_at = os.date('!%Y-%m-%dT%H:%M:%SZ')
        snapshot.aircraft = snapshot.actor -- retained for older analysis scripts
        snapshot.targets = new_array()
        local candidates = {}
        snapshot.scanned_count, snapshot.scan_error = each(FindAllOf('LiveGameObject'), 20000, function(actor)
            if valid(actor) then
                local name = call(call(call(actor, 'GetClass'), 'GetFName'), 'ToString')
                local lower = name and name:lower() or ''
                if lower:find('ladn', 1, true) or lower:find('ladon', 1, true) or lower:find('x40a', 1, true) then
                    candidates[#candidates + 1] = { actor = actor, class = name }
                end
            end
        end)
        -- Prefer the main LADON actor before individually lockable parts.
        table.sort(candidates, function(a, b)
            local am = a.class:lower():find('ladn_cp_c', 1, true) ~= nil or a.class:lower():find('_cp_tewy', 1, true) ~= nil
            local bm = b.class:lower():find('ladn_cp_c', 1, true) ~= nil or b.class:lower():find('_cp_tewy', 1, true) ~= nil
            if am ~= bm then return am end
            return a.class < b.class
        end)
        snapshot.target_count = #candidates
        snapshot.targets_truncated = #candidates > 16
        for i = 1, math.min(#candidates, 16) do
            local actor = candidates[i].actor
            local item_ok, item = pcall(actor_snapshot, actor, actor_component)
            snapshot.targets[#snapshot.targets + 1] = item_ok and item
                or { actor = full_name(actor), class = candidates[i].class, error = tostring(item) }
        end
        local path = directory .. 'mesh-probe-' .. os.date('%Y%m%d-%H%M%S') .. '.json'
        local file = assert(io.open(path, 'w'))
        file:write(json(snapshot), '\n')
        file:close()
        print('[AC8MouseAim] Mesh probe saved: ' .. path .. '\n')
    end)
    if not ok then print('[AC8MouseAim] Mesh probe failed: ' .. tostring(err) .. '\n') end
end

return M
