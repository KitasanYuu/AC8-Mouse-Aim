-- One-shot, read-only probe of the aircraft parameter tables the game has loaded (F7): writes a
-- JSON snapshot beside the script. The hangar shows Speed, Maneuverability and Stability as bars
-- with no numbers (and they are not progress-bar widgets); the per-aircraft parameter tables
-- (DT_PlayerParameter_<aircraft>) and the hangar's aircraft tables are read instead. It does not
-- inspect game archives or keep UObject handles.
--
-- Two steps, no restart between them. Without spec-fields.txt beside the script, F7 has UE4SS
-- write its object dump (every type's fields); the field list is made from it outside the game.
-- With it, F7 reads those fields of every row.
--
-- Safety: the first version crashed the game in UE4SS on a null read. Reading a struct-typed
-- field of a data-table row makes UE4SS turn the whole struct into a Lua table, recursively, with
-- no owning object, and some field types then dereference it; and this UE4SS gives Lua no way
-- to list a row struct's fields. So only listed fields are read, all plain (numbers, booleans,
-- names, strings, enums) or plain structs. One table at a time, the file rewritten and a log line
-- written after each, so a crash shows where it stopped.
local M = {}
local WANTED = { 'DT_PlayerParameter_', 'DT_LiveAircraft.', 'DT_LobbyDisplayAircraftOverview', 'DT_HangarAircraft',
                 'DT_OnlineAircraftParam', 'DT_AircraftRole' }

local function call(obj, name, ...)
    if not obj then return nil end
    local ok, value = pcall(function(...) return obj[name](obj, ...) end, ...)
    return ok and value or nil
end

local function get(obj, key)
    if not obj then return nil end
    local ok, value = pcall(function() return obj[key] end)
    return ok and value or nil
end

local function valid(obj)
    local ok, result = pcall(function() return obj and obj:IsValid() end)
    return ok and result == true
end

local function full_name(obj)
    if not valid(obj) then return nil end
    local ok, name = pcall(function() return obj:GetFullName() end)
    return ok and type(name) == 'string' and name or nil
end

local function text_of(value)
    if value == nil then return nil end
    if type(value) == 'string' then return value end
    local ok, s = pcall(function() return value:ToString() end)
    return ok and type(s) == 'string' and s or nil
end

-- A value as JSON can hold it: numbers, booleans, strings; FName/FString userdata as text;
-- Lua tables (a plain struct UE4SS converted) field by field.
local function plain(value, depth)
    local kind = type(value)
    if kind == 'number' or kind == 'boolean' or kind == 'string' then return value end
    if kind == 'table' then
        if (depth or 0) > 6 then return nil end
        local out = {}
        for k, v in pairs(value) do out[tostring(k)] = plain(v, (depth or 0) + 1) end
        return out
    end
    if kind == 'userdata' then
        local ok, n = pcall(function() return value:get() end)
        if ok and (type(n) == 'number' or type(n) == 'boolean') then return n end
        return text_of(value)
    end
    return nil
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

local function array() return setmetatable({}, { __json_array = true }) end

-- The fields to read, prepared outside the game from UE4SS's object dump: lines
-- "<row struct name>\t<field>[.<field>...]", plain fields only (spec-fields.txt beside the script);
-- "table\t<part of a table's name>" adds a table to read, and "class\t<class>\t<property>" reads
-- that property of every live object of that class (the player's gun rounds in a mission, say),
-- one entry per class (new ones need no restart either). "world\t1" also reads the level's units
-- (read_world: a mission's spawn layout, 2026-10-06; struct arrays, so read last, saved after each).
local CLASSES = {}   -- class name -> properties to read (from "class" lines)
local WORLD = false  -- a "world" line: the level's units too (read_world)
local function load_fields(directory)
    local file = io.open(directory .. 'spec-fields.txt', 'r')
    if not file then return nil end
    local fields = {}
    WORLD = false
    for line in file:lines() do
        local cls, prop = line:match('^class\t([^\t]+)\t([^\t]+)$')
        if cls then
            CLASSES[cls] = CLASSES[cls] or {}
            local known = false
            for _, p in ipairs(CLASSES[cls]) do if p == prop then known = true end end
            if not known then CLASSES[cls][#CLASSES[cls] + 1] = prop end
        end
        local struct, path = line:match('^([^\t#]+)\t([^\t]+)$')
        if struct == 'world' then
            WORLD = true
        elseif struct == 'table' then
            local known = false
            for _, word in ipairs(WANTED) do if word == path then known = true end end
            if not known then WANTED[#WANTED + 1] = path end
        elseif struct then
            fields[struct] = fields[struct] or {}
            local parts = {}
            for part in path:gmatch('[^%.]+') do parts[#parts + 1] = part end
            fields[struct][#fields[struct] + 1] = { path = path, parts = parts }
        end
    end
    file:close()
    return fields
end

-- A field down a path: a number steps into an array (1-based), a name into a struct. An array
-- is indexed only within its length: UE4SS's TArray adds zeroed elements when indexed past the
-- end (a read once asked for levels 1-10 and may have grown a table in memory, 2026-10-05).
local function step(value, part)
    local index = tonumber(part)
    if type(value) == 'table' then return value[index or part] end
    if type(value) ~= 'userdata' then return nil end
    if index then
        local ok, length = pcall(function() return value:GetArrayNum() end)
        if not ok or type(length) ~= 'number' or index < 1 or index > length then return nil end
    end
    local ok, item = pcall(function() return value[index or part] end)
    return ok and item or nil
end

-- The level's units: every LiveWorldData asset loaded (a mission's global data and its phases,
-- WD_Mission<nn>_phase<k>), each unit in it (UnitDescList: where it starts, whether from the
-- start, its height band and patrol) and each aircraft of the unit (AIObjectDescList: its mission
-- NPC id, class, start). Plain fields by name only, and the vectors' components one by one; arrays
-- within their length. The other arrays (tags, child objects) are not touched.
local UNIT_FIELDS = { 'bIsSpawnFromStart', 'Faction', 'MinAltitudeInMeter', 'MaxAltitudeInMeter', 'RadiusInMeter',
                      'PatrolRadiusRatio', 'DisplayName', 'RandomGroupIndex', 'bIsEscortedUnit', 'ParentWingmanName' }
local OBJECT_FIELDS = { 'MissionNpcID', 'TargetMarkType', 'bIsInvincible', 'GameObjectDisplayNameID02', 'AITypeId',
                        'bGroupLead', 'bNotGroup', 'VariationID', 'AttachParentAIObjectName', 'bIsLanding',
                        'bShouldShowInBriefing', 'bIsTgtInBriefing' }

local function length(arr)
    local ok, n = pcall(function() return arr:GetArrayNum() end)
    return ok and type(n) == 'number' and n or 0
end

local function components(value, keys)
    if value == nil then return nil end
    local out = {}
    for _, key in ipairs(keys) do
        local x = plain(get(value, key))
        if type(x) == 'number' then out[key] = x end
    end
    return out
end

local function fields_of(item, names)
    local out = {}
    for _, name in ipairs(names) do
        local v = plain(get(item, name))
        if v ~= nil then out[name] = v end
    end
    return out
end

local function read_world(obj, log)
    local out = { table = full_name(get(obj, 'NpcMissionDataTable')), units = array() }
    local list = get(obj, 'UnitDescList')
    local n = math.min(length(list), 1000)
    for i = 1, n do
        local unit = step(list, i)
        if unit ~= nil then
            local u = fields_of(unit, UNIT_FIELDS)
            u.at = components(get(unit, 'SpawnPoint'), { 'X', 'Y', 'Z' })
            u.rotation = components(get(unit, 'Rotation'), { 'Pitch', 'Yaw', 'Roll' })
            u.aircraft = array()
            local objects = get(unit, 'AIObjectDescList')
            for k = 1, math.min(length(objects), 200) do
                local item = step(objects, k)
                if item ~= nil then
                    local a = fields_of(item, OBJECT_FIELDS)
                    a.class = full_name(get(item, 'AIObjectClass'))
                    a.at = components(get(item, 'SpawnPoint'), { 'X', 'Y', 'Z' })
                    a.rotation = components(get(item, 'Rotation'), { 'Pitch', 'Yaw', 'Roll' })
                    u.aircraft[#u.aircraft + 1] = a
                end
            end
            out.units[#out.units + 1] = u
        end
    end
    out.unit_count = n
    return out
end

local function read_path(row, parts)
    local value = get(row, parts[1])
    for i = 2, #parts do
        value = step(value, parts[i])
        if value == nil then return nil end
    end
    return plain(value)
end

function M.run(directory, notice)
    local function log(message) if notice then notice('Spec probe: ' .. message) end end
    local fields = load_fields(directory)
    if not fields then
        -- step 1: the dump of every type's fields (UE4SS's own "Dump Objects & Properties")
        local ok, err = pcall(function()
            local function dump() DumpAllObjects(); log('object dump written (UE4SS_ObjectDump.txt); no spec-fields.txt yet') end
            if type(ExecuteInGameThread) == 'function' then ExecuteInGameThread(dump) else dump() end
        end)
        if not ok then log('dump failed: ' .. tostring(err)) end
        return
    end
    local path = directory .. 'spec-probe-' .. os.date('%Y%m%d-%H%M%S') .. '.json'
    local result = { version = 4, captured_at = os.date('!%Y-%m-%dT%H:%M:%SZ'), stage = 'start', tables = array() }
    local function save(stage)
        result.stage = stage
        local file = io.open(path, 'w')
        if file then file:write(json(result), '\n'); file:close() end
        log(stage)
    end
    local function work()
        save('start')
        local targets = {}
        for _, value in ipairs(FindAllOf('DataTable') or {}) do
            local ok, t = pcall(function() return value:get() end)
            t = ok and t or value
            local name = full_name(t)
            if name and not name:find('Default__', 1, true) and not name:find('/Engine/Transient', 1, true) then
                for _, word in ipairs(WANTED) do
                    if name:find(word, 1, true) then targets[#targets + 1] = { table = t, name = name }; break end
                end
            end
        end
        save(string.format('%d tables to read', #targets))
        for index, target in ipairs(targets) do
            local short = target.name:match('([^%.]+)$') or target.name
            local struct_name = (full_name(call(target.table, 'GetRowStruct')) or ''):match('([^%./]+)$') or ''
            local list = fields[struct_name] or {}
            log(string.format('reading %d/%d %s (%s, %d fields)', index, #targets, short, struct_name, #list))
            local item = { name = target.name, row_struct = struct_name, rows = {} }
            result.tables[#result.tables + 1] = item
            local count = 0
            if #list > 0 then
                pcall(function()
                    target.table:ForEachRow(function(row_name, row)
                        count = count + 1
                        if count > 2000 then return true end
                        local values = {}
                        for _, field in ipairs(list) do
                            local ok, value = pcall(read_path, row, field.parts)
                            if ok and value ~= nil then values[field.path] = value end
                        end
                        item.rows[text_of(row_name) or tostring(count)] = values
                        return nil
                    end)
                end)
            end
            item.row_count = count
            save(string.format('read %d/%d %s (%d rows)', index, #targets, short, count))
        end
        -- live objects of the listed classes: plain properties by name, one entry per object class
        -- (instances and class defaults alike: same values unless set at run time)
        result.classes = {}
        for cls, props in pairs(CLASSES) do
            log('reading class ' .. cls)
            local per = {}
            local seen = 0
            for _, value in ipairs(FindAllOf(cls) or {}) do
                seen = seen + 1
                if seen > 500 then break end
                local ok, obj = pcall(function() return value:get() end)
                obj = ok and obj or value
                local name = full_name(obj)
                local kind = full_name(call(obj, 'GetClass')) or '?'
                if name then
                    local entry = per[kind]
                    if not entry then
                        entry = { count = 0, example = name, values = {} }
                        for _, prop in ipairs(props) do
                            local v = plain(get(obj, prop))
                            if v ~= nil then entry.values[prop] = v end
                        end
                        per[kind] = entry
                    end
                    entry.count = entry.count + 1
                end
            end
            result.classes[cls] = per
            save(string.format('class %s: %d objects', cls, seen))
        end
        if WORLD then
            result.worlds = array()
            local assets = FindAllOf('LiveWorldData') or {}
            save(string.format('%d world data assets to read', #assets))
            for index, value in ipairs(assets) do
                local ok, obj = pcall(function() return value:get() end)
                obj = ok and obj or value
                local name = full_name(obj)
                if name and not name:find('Default__', 1, true) then
                    log(string.format('reading world %d/%d %s', index, #assets, name:match('([^%.]+)$') or name))
                    local okr, w = pcall(read_world, obj, log)
                    if okr and w then w.name = name; result.worlds[#result.worlds + 1] = w end
                    save(string.format('world %d/%d: %s', index, #assets, okr and (w.unit_count .. ' units') or tostring(w)))
                end
            end
        end
        save(string.format('done: %d tables', #result.tables))
    end
    local ok, err = pcall(function()
        if type(ExecuteInGameThread) == 'function' then ExecuteInGameThread(work) else work() end
    end)
    if not ok then log('failed: ' .. tostring(err)) end
end

return M
