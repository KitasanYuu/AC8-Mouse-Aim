local script = 'Payload/Game/Binaries/Win64/UE4SS/Mods/AC8MouseAim/Scripts/mesh_probe.lua'
local probe = dofile(script)

local function object(name, fields)
    local result = fields or {}
    function result:IsValid() return true end
    function result:GetFullName() return name end
    return result
end

local body = object('BodySetup test', {
    AggGeom = { ConvexElems = { { VertexData = { { X = 100, Y = 0, Z = 0 },
        { X = 0, Y = 200, Z = 0 }, { X = 0, Y = 0, Z = 300 } },
        IndexData = { 0, 1, 2 } } }, BoxElems = {}, SphereElems = {}, SphylElems = {} },
})
local mesh = object('StaticMesh test', { BodySetup = body })
local component = object('StaticMeshComponent test', {
    StaticMesh = mesh,
    RelativeLocation = { X = 1, Y = 2, Z = 3 },
    RelativeRotation = { Pitch = 0, Yaw = 90, Roll = 0 },
    RelativeScale3D = { X = 1, Y = 1, Z = 1 },
})
function component:GetClass() return object('Class StaticMeshComponent') end
local pawn = object('PlayerPlane test')
function pawn:K2_GetComponentsByClass()
    return { { get = function() return component end } }
end
StaticFindObject = function() return object('Class ActorComponent') end
local ladon = object('LADON instance')
function ladon:GetClass()
    local cls = object('Class BP_OP0045_ladn_CP_C')
    function cls:GetFName() return { ToString = function() return 'BP_OP0045_ladn_CP_C' end } end
    return cls
end
function ladon:K2_GetComponentsByClass() return { component } end
FindAllOf = function() return { ladon } end

local written = ''
local original_open = io.open
io.open = function()
    return { write = function(_, value) written = written .. value end, close = function() end }
end
probe.request()
probe.update(pawn, '')
local manual = written
-- automatic (telemetry on): nothing until the aircraft has flown 5 s, then once only
function pawn:GetClass()
    local cls = object('Class BP_PlayerPlane_test_C')
    function cls:GetFName() return { ToString = function() return 'BP_PlayerPlane_test_C' end } end
    return cls
end
FindFirstOf = function() return nil end
local clock, original_clock = 100, os.clock
os.clock = function() return clock end
written = ''
probe.update(pawn, '', false)
assert(written == '', 'snapshot while telemetry is off')
probe.update(pawn, '', true)
assert(written == '', 'snapshot before the aircraft flew 5 s')
clock = 106
probe.update(pawn, '', true)
assert(written ~= '', 'no automatic snapshot of the aircraft')
written = ''
clock = 200
probe.update(pawn, '', true)
assert(written == '', 'the same aircraft taken twice')
-- a LADON appearing: taken once
FindFirstOf = function(name) return name == 'BP_OP0045_ladn_CP_C' and ladon or nil end
clock = 300
probe.update(pawn, '', true)
assert(written ~= '', 'no automatic snapshot when LADON appeared')
written = ''
clock = 400
probe.update(pawn, '', true)
assert(written == '', 'LADON taken twice')
os.clock = original_clock
io.open = original_open
written = manual

assert(written:find('"mesh":"StaticMesh test"', 1, true))
assert(written:find('"translation":[1,2,3]', 1, true))
assert(written:find('"vertices":[[100,0,0],[0,200,0],[0,0,300]]', 1, true))
assert(written:find('"indices":[0,1,2]', 1, true))
assert(written:find('"target_count":1', 1, true))
assert(written:find('"actor":"LADON instance"', 1, true))
print('Mesh probe checks passed.')
