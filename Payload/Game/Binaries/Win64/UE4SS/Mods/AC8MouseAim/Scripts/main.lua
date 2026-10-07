-- 0.2.30: game-thread numeric bridge; no realtime files or named pipes.
local directory = assert(debug.getinfo(1, "S").source:sub(2):match("^(.*[/\\])"))
local aim_camera = dofile(directory .. "camera.lua")
local gaze_probe = dofile(directory .. "gaze_probe.lua")
local mesh_probe = dofile(directory .. "mesh_probe.lua")
local spec_probe = dofile(directory .. "spec_probe.lua")
local gaze = dofile(directory .. "gaze.lua")
local start_native = assert(package.loadlib(directory .. "ac8_mouse_aim_010.dll", "ac8_mouseaim_start"))
local begin_native = assert(package.loadlib(directory .. "ac8_mouse_aim_010.dll", "ac8_mouseaim_begin"))
local frame_native = assert(package.loadlib(directory .. "ac8_mouse_aim_010.dll", "ac8_mouseaim_frame"))
local camera_native = assert(package.loadlib(directory .. "ac8_mouse_aim_010.dll", "ac8_mouseaim_camera"))
local release_native = assert(package.loadlib(directory .. "ac8_mouse_aim_010.dll", "ac8_mouseaim_release"))
local requests_native = assert(package.loadlib(directory .. "ac8_mouse_aim_010.dll", "ac8_mouseaim_requests"))
-- Optional: other aircraft for the telemetry stream (contacts.lua).
local send_native = package.loadlib(directory .. "ac8_mouse_aim_010.dll", "ac8_mouseaim_send")
local contacts = dofile(directory .. "contacts.lua")
if send_native then contacts.init(send_native) end
local current_address = nil
local startup_address, startup_time = nil, 0
local next_search = 0
local reported_rotation_shape = false
local pause_gameplay
local rotation_fields={}

local function unwrap_number(value)
    if type(value) == "number" then return value end
    if type(value) == "table" or type(value) == "userdata" then
        local ok_get, getter = pcall(function() return value.get end)
        if ok_get and type(getter) == "function" then
            local ok_value, unwrapped = pcall(function() return value:get() end)
            if ok_value and type(unwrapped) == "number" then return unwrapped end
        end
    end
    return nil
end

local function rotation_component(rotation, wanted)
    -- Cache discovered spellings (AC exposes pitch/Yaw/Roll) instead of
    -- enumerating the same tables for each component every frame.
    if type(rotation)=='table' and rotation_fields[wanted] then
        local value=unwrap_number(rotation[rotation_fields[wanted]])
        if value then return value end
    end
    local ok_direct, direct = pcall(function() return rotation[wanted] end)
    if ok_direct then
        local value = unwrap_number(direct)
        if value then rotation_fields[wanted]=wanted; return value end
    end

    if type(rotation) == "table" then
        local wanted_lower = string.lower(wanted)
        for key, candidate in pairs(rotation) do
            if type(key) == "string" and string.find(string.lower(key), wanted_lower, 1, true) then
                local value = unwrap_number(candidate)
                if value then rotation_fields[wanted]=key; return value end
            end
        end
    end
    return nil
end

local function describe_rotation(rotation)
    if type(rotation) ~= "table" then return type(rotation) end
    local fields = {}
    for key, value in pairs(rotation) do
        fields[#fields + 1] = tostring(key) .. "=" .. type(value)
    end
    table.sort(fields)
    return "table{" .. table.concat(fields, ",") .. "}"
end
local last_notice = nil

local function notice(message)
    if message ~= last_notice then
        print("[AC8MouseAim] " .. message .. "\n")
        last_notice = message
    end
end

local plane_class,cached_controller
local function player_plane()
    if not plane_class or not plane_class:IsValid() then
        plane_class=StaticFindObject("/Script/Live.LivePlayerPlane")
    end
    if not plane_class or not plane_class:IsValid() then return nil end
    -- Reuse the controller throughout gameplay; read Pawn again each frame so
    -- respawn/possession changes still reach the existing lifecycle handling.
    if cached_controller and cached_controller:IsValid() then
        local pawn=cached_controller.Pawn
        if pawn and pawn:IsValid() and pawn:IsA(plane_class) then return pawn,cached_controller end
    end
    cached_controller=nil
    local selected, selected_controller, count = nil, nil, 0
    for _, controller in ipairs(FindAllOf("LivePlayerController") or {}) do
        if controller:IsValid() then
            local pawn = controller.Pawn
            if pawn and pawn:IsValid() and pawn:IsA(plane_class) then
                selected = pawn
                selected_controller = controller
                count = count + 1
            end
        end
    end
    if count == 1 then cached_controller=selected_controller; return selected, selected_controller end
    return nil, nil
end

-- The player model's detail (issue #7, a lower-detail model reported with the mod's camera):
-- once per aircraft, the body mesh's LOD switch points (each LOD's screen size, the mesh's bounds)
-- and the engine's LOD settings; every 3 s, when it changes, the LOD the engine picks, the camera's
-- distance from the aircraft and the FOV. Into UE4SS.log; read-only.
local lod_watch = { pawn = nil, mesh = nil, next = 0, last = nil }
local function lod_settings(mesh)
    local parts = {}
    local asset
    for _, get in ipairs({ function() return mesh:GetSkeletalMeshAsset() end, function() return mesh.SkeletalMesh end,
                           function() return mesh.SkinnedAsset end }) do
        local ok, value = pcall(get)
        if ok and value and value:IsValid() then asset = value; break end
    end
    if asset then
        local sizes = {}
        pcall(function()
            asset.LODInfo:ForEach(function(_, element)
                local okv, info = pcall(function() return element:get() end)
                if not okv or not info then info = element end
                local oks, size = pcall(function() return info.ScreenSize.Default end)
                sizes[#sizes + 1] = oks and string.format('%.3f', tonumber(size) or -1) or '?'
            end)
        end)
        parts[#parts + 1] = 'screen sizes [' .. table.concat(sizes, ' ') .. ']'
        pcall(function() parts[#parts + 1] = 'mesh min LOD ' .. tostring(asset.MinLod.Default) end)
    else
        parts[#parts + 1] = 'mesh asset unreadable'
    end
    pcall(function() parts[#parts + 1] = string.format('bounds radius %.0f cm', mesh.Bounds.SphereRadius) end)
    pcall(function() parts[#parts + 1] = 'forced ' .. tostring(mesh.ForcedLodModel) .. ' min ' .. tostring(mesh.MinLodModel) ..
        ' override min ' .. tostring(mesh.bOverrideMinLod) end)
    local system = StaticFindObject('/Script/Engine.Default__KismetSystemLibrary')
    if system and system:IsValid() then
        for _, name in ipairs({ 'r.SkeletalMeshLODBias', 'r.SkeletalMeshLODRadiusScale', 'r.ViewDistanceScale',
                                'r.StaticMeshLODDistanceScale', 'sg.ViewDistanceQuality', 'r.ScreenPercentage' }) do
            local ok, value = pcall(function() return system:GetConsoleVariableFloatValue(name) end)
            parts[#parts + 1] = name .. '=' .. (ok and tostring(value) or '?')
        end
    end
    return table.concat(parts, ', ')
end
local function watch_lod(pawn, ox, oy, oz, fov)
    local now = os.clock()
    if now < lod_watch.next then return end
    lod_watch.next = now + 3
    local address = pawn:GetAddress()
    if lod_watch.pawn ~= address then
        lod_watch.pawn = address; lod_watch.mesh = nil; lod_watch.last = nil
        local ok, mesh = pcall(function() return pawn.PlaneBodyMesh end)
        if ok and mesh and mesh:IsValid() then lod_watch.mesh = mesh end
        if not lod_watch.mesh then print('[AC8MouseAim] LOD watch: no PlaneBodyMesh found\n'); return end
        local okd, detail = pcall(lod_settings, lod_watch.mesh)
        print('[AC8MouseAim] LOD watch ' .. (pawn:GetFullName():match('^(%S+)') or '?') .. ': ' .. (okd and detail or tostring(detail)) .. '\n')
    end
    local mesh = lod_watch.mesh
    if not mesh or not mesh:IsValid() then return end
    local ok, lod = pcall(function() return mesh:GetPredictedLODLevel() end)
    if not ok then print('[AC8MouseAim] LOD watch: ' .. tostring(lod) .. '\n'); lod_watch.next = now + 60; return end
    if type(lod) ~= 'number' then local okv, v = pcall(function() return lod:get() end); if okv then lod = v end end
    local line = string.format('LOD body=%s camera %.0f m from the aircraft, FOV %.0f', tostring(lod),
        math.sqrt(ox * ox + oy * oy + oz * oz) / 100, tonumber(fov) or -1)
    if line ~= lod_watch.last then print('[AC8MouseAim] ' .. line .. '\n'); lod_watch.last = line end
end

local function camera_rotation(manager, fallback)
    local ok_camera, rotation = pcall(function() return manager:GetCameraRotation() end)
    if ok_camera and rotation then return rotation end
    local ok_actor, actor_rotation = pcall(function() return manager:K2_GetActorRotation() end)
    if ok_actor and actor_rotation then return actor_rotation end
    return fallback
end

-- No keys are bound here: config.ini [keys] has them all and the native side watches them (reload
-- and performance counters it carries out itself); the ones done here come as requests each frame.
local function carry_out_requests()
    local requests = tonumber(requests_native()) or 0
    if requests % 2 == 1 then gaze_probe.request() end                                       -- camera_probe
    if math.floor(requests / 2) % 2 == 1 then pcall(spec_probe.run, directory, notice) end   -- hangar_specs
    if math.floor(requests / 4) % 2 == 1 then pcall(mesh_probe.hangar, directory) end        -- hangar_geometry
end

assert(start_native(1729,0.125)==30,'AC8 direct bridge unavailable; control disabled (check native log).')

if EngineTickAvailable == false or type(LoopInGameThreadAfterFrames) ~= "function" then
    notice("Disabled: required game-thread callback unavailable.")
else
    LoopInGameThreadAfterFrames(1, function()
        begin_native()
        pcall(carry_out_requests)
        local ok, err = pcall(function()
            local now = os.time()
            if now < next_search then return end
            local pawn, controller = player_plane()
            if not pawn then
                gaze_probe.end_mission()
                startup_address=nil; startup_time=0
                aim_camera.restore()
                if current_address then
                    release_native()
                    current_address = nil
                end
                next_search = now + 1
                notice("Waiting for a single-player aircraft.")
                return
            end
            local incoming_address=pawn:GetAddress()
            if startup_address~=incoming_address then
                aim_camera.restore()
                release_native()
                current_address=nil
                startup_address=incoming_address
                contacts.reset()
                startup_time=0
            end
            if not pause_gameplay or not pause_gameplay:IsValid() then
                pause_gameplay=StaticFindObject('/Script/Engine.Default__GameplayStatics')
            end
            local dt=pause_gameplay:GetWorldDeltaSeconds(pawn)
            -- Allow the spawned pawn's mission transform to replace construction defaults.
            if startup_time<0.5 then
                startup_time=startup_time+math.max(0,math.min(0.1,dt))
                return
            end
            gaze_probe.update(pawn,controller,directory)
            mesh_probe.update(pawn,directory,contacts.active())   -- automatic while recording
            local rotation = pawn:K2_GetActorRotation()
            assert(rotation, "K2_GetActorRotation returned nil")
            local pitch = rotation_component(rotation, "Pitch")
            local yaw = rotation_component(rotation, "Yaw")
            local roll = rotation_component(rotation, "Roll")
            if not pitch or not yaw or not roll then
                if not reported_rotation_shape then
                    reported_rotation_shape = true
                    notice("Unsupported rotation value: " .. describe_rotation(rotation))
                end
                error("Unable to read aircraft Pitch/Yaw/Roll")
            end
            local manager=controller.PlayerCameraManager
            assert(manager and manager:IsValid(),'Camera manager unavailable')
            local gazing=gaze.update(pawn,pause_gameplay:GetRealTimeSeconds(pawn))
            -- Scripted scenes (mission-end replay) hand the camera and controls back too.
            if gaze.scene(pawn,controller) then gazing=true end
            local camera = camera_rotation(manager, rotation)
            local camera_pitch = rotation_component(camera, "Pitch") or pitch
            local camera_yaw = rotation_component(camera, "Yaw") or yaw
            local camera_roll = rotation_component(camera, "Roll") or roll
            local address = incoming_address
            assert(type(address) == "number" and address > 0, "Invalid aircraft address")
            local fov=100
            pcall(function() fov=manager:GetFOVAngle() end)
            local position=pawn:K2_GetActorLocation()
            local view_position=manager:GetCameraLocation()
            local ox=assert(rotation_component(view_position,"X"))-assert(rotation_component(position,"X"))
            local oy=assert(rotation_component(view_position,"Y"))-assert(rotation_component(position,"Y"))
            local oz=assert(rotation_component(view_position,"Z"))-assert(rotation_component(position,"Z"))
            local paused=pause_gameplay:IsGamePaused(pawn)
            -- The game's own throttle/brake input (both held, or the single high-G button,
            -- is a high-G turn), whatever the binding.
            local throttle,brake=0,0
            pcall(function() throttle=tonumber(pawn.InputThrottle) or 0 end)
            pcall(function() brake=tonumber(pawn.InputBrake) or 0 end)
            -- The player's own flight input, as the game has it from its bindings (keyboard or
            -- gamepad): it takes over an axis; both yaw inputs held is the game's autopilot (Q+E, or
            -- the autopilot key, which the game turns into both).
            local input_pitch,input_roll,input_left,input_right=0,0,0,0
            pcall(function() input_pitch=tonumber(pawn.InputPitch) or 0 end)
            pcall(function() input_roll=tonumber(pawn.InputRoll) or 0 end)
            pcall(function() input_left=tonumber(pawn.InputLeftYaw) or 0 end)
            pcall(function() input_right=tonumber(pawn.InputRightYaw) or 0 end)
            -- The game's world clock: positions and attitudes advance by game frames.
            local game_time=-1
            pcall(function() game_time=tonumber(pause_gameplay:GetTimeSeconds(pawn)) or -1 end)
            local on,target_pitch,target_yaw,follow=frame_native(address,pitch,yaw,roll,
                camera_pitch,camera_yaw,camera_roll,fov,ox,oy,oz,paused and 1 or 0,gazing and 1 or 0,
                throttle,brake,
                assert(rotation_component(position,"X")),assert(rotation_component(position,"Y")),
                assert(rotation_component(position,"Z")),game_time,input_pitch,input_roll,input_left,input_right)
            assert(on~=nil,'Native frame rejected')
            pcall(contacts.update,pawn,game_time)
            pcall(watch_lod,pawn,ox,oy,oz,fov)
            local desired_camera
            if gazing then
                aim_camera.seed(camera,rotation_component)
            else
                desired_camera=aim_camera.update(pawn,controller,rotation,rotation_component,
                    on,target_pitch,target_yaw,dt,follow)
            end
            if desired_camera then
                assert(camera_native(manager:GetAddress(),address,
                    desired_camera.pitch,desired_camera.yaw,desired_camera.roll)==1,'Camera command rejected')
            else
                camera_native(0,0,0,0,0)
            end
            if current_address ~= address then
                current_address = address
                notice("Active for " .. pawn:GetFullName())
            end
        end)
        if not ok then
            release_native()
            current_address = nil
            next_search = os.time() + 1
            notice("Recovering after runtime error: " .. tostring(err))
        end
    end)
    notice("Loaded. Offline controller will activate after entering a mission.")
end
