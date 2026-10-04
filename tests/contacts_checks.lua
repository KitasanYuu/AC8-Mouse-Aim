-- contacts.lua against stand-in game objects (UE4SS objects are not available here):
-- scans and samples run without errors, elites and enemies are tracked first, the
-- player's class is reported, a boss with an unusual class is picked up, and an aircraft
-- that stops flying is reported in "gone".
local objects = {}
local function object(id, class, x, y, z, opts)
    opts = opts or {}
    local o = {id = id, class = class, x = x, y = y, z = z, hidden = opts.hidden or false,
               target = opts.target or false, speed = opts.speed or 200, valid = true}
    o.IsValid = function(self) return self.valid end
    o.GetAddress = function(self) return self.id end
    o.GetClass = function(self)
        return {GetFName = function() return {ToString = function() return self.class end} end}
    end
    o.K2_GetActorLocation = function(self) return {X = self.x * 100, Y = self.y * 100, Z = self.z * 100} end
    o.K2_GetActorRotation = function() return {Pitch = 0, Yaw = 10, Roll = 0} end
    o.GetSpeedMps = function(self) return self.speed end
    o.IsPlayerTarget = function(self) return self.target end
    setmetatable(o, {__index = function(t, k) if k == 'bHidden' then return rawget(t, 'hidden') end end})
    objects[#objects + 1] = o
    return o
end
local pawn = object(1, 'BP_PlayerPlane_PP0021_su57_C', 0, 0, 3000)
pawn.TargetSelectionComponent = {GetSelectedTarget = function() return nil end}
local enemy = object(2, 'BP_OP1017_mr2k_CP_C', 900, 0, 3000, {target = true})
object(3, 'BP_OP1020_m29a_CP_Shadow_C', 9000, 0, 3000, {target = true})
object(4, 'BP_WOP1004_f15c_C', 300, 0, 3000)
object(5, 'BP_Boss_Something_C', 4000, 0, 4000, {target = true, speed = 250})
object(9, 'BP_OP0045_ladn_CP_C', 20000, 0, 9000, {speed = 600})
object(6, 'BP_GR0001_tank_C', 100, 0, 0, {target = true, speed = 15})
object(7, 'BP_PlayerPlane_PP0004_f15c_C', 0, 0, 0, {hidden = true})
-- a lockable part of a boss: no speed of its own, moves with the aircraft
local part = object(8, 'BP_Boss_Part_Engine_C', 5000, 0, 5000, {target = true, speed = 0})
FindAllOf = function() return objects end

local packets = {}
local contacts = dofile('Payload/Game/Binaries/Win64/UE4SS/Mods/AC8MouseAim/Scripts/contacts.lua')
contacts.init(function(text) packets[#packets + 1] = text; return 1 end)
contacts.update(pawn, 1.0)
assert(packets[1], 'no packet')
assert(packets[1]:find('"classes"'), 'class census missing on a scan')
-- the aircraft list alone (the class census at the end names every class seen)
local first = packets[1]:match('"c":%[(.*)%]')
first = first:sub(1, (first:find('"classes"') or #first + 1) - 1)
assert(packets[1]:find('"self":"BP_PlayerPlane_PP0021_su57_C"'), 'player class missing')
assert(first:find('BP_OP1020_m29a_CP_Shadow_C'), 'elite not tracked')
assert(first:find('BP_Boss_Something_C'), 'unusual hostile class not tracked')
assert(not first:find('BP_GR0001_tank_C'), 'ground unit tracked')
assert(not first:find('PP0004'), 'pooled plane tracked')
-- the Shadow comes before nearer ordinary enemies, enemies before the wingman
local shadow, mr2k, wingman = first:find('Shadow'), first:find('mr2k'), first:find('WOP1004')
assert(shadow < mr2k and mr2k < wingman, 'tracking order')
assert(not first:find('Engine'), 'a still part taken for an aircraft')
-- a LADON (not a player target itself) is tracked first despite being farthest
assert(first:find('ladn_CP_C') < first:find('mr2k'), 'LADON not ranked as an elite')
-- the part moves 400 m/s with its aircraft: picked up at the next scan (3 s on)
part.x = part.x + 1200
contacts.update(pawn, 4.1)
assert(packets[2]:find('BP_Boss_Part_Engine_C'), 'moving part not tracked')
packets = {packets[1]}
-- shot down: hidden, then reported once as gone
enemy.hidden = true
contacts.update(pawn, 4.3)
assert(packets[2]:find('"gone":%[2%]'), 'gone not reported')
contacts.update(pawn, 4.5)
assert(not packets[3]:find('"gone"'), 'gone reported twice')
print('Lua contacts checks passed')
