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
-- AC8's rotator spells pitch in lower case
enemy.K2_GetActorRotation = function() return {pitch = 12.5, Yaw = 10, Roll = -30} end
object(3, 'BP_OP1020_m29a_CP_Shadow_C', 9000, 0, 3000, {target = true})
object(4, 'BP_WOP1004_f15c_C', 300, 0, 3000)
object(5, 'BP_Boss_Something_C', 4000, 0, 4000, {target = true, speed = 250})
object(10, 'BP_OP1016_x40a_CP_Tewy_C', 30000, 0, 5000, {speed = 500})   -- Moon 11: a boss, far, not flagged
object(11, 'BP_OP1016_x40a_RG_CP_C', 30000, 0, 5000, {speed = 500})     -- its part, not a boss
object(12, 'BP_OP0024_uavn_CP_C', 30050, 0, 5000, {speed = 500})        -- her shield drone, an escort
local ladon = object(9, 'BP_OP0045_ladn_CP_C', 20000, 0, 9000, {speed = 600})
-- its skeleton (component space, cm; as the mesh probe logged it: swept 66 deg, folded 36 deg)
FName = function(name) return name end
StaticFindObject = function() return {} end
local wing_bones = {
    BN_L_VariableWing_01 = {930, -941, -29}, BN_L_VariableWing_02 = {-3658, -2988, -11.5}, BN_L_VariableWing_04 = {-4306, -3355, -540},
    BN_R_VariableWing_01 = {930, 941, -29}, BN_R_VariableWing_02 = {-3658, 2988, -11.5}, BN_R_VariableWing_04 = {-4306, 3355, -540},
}
ladon.Mesh = {
    GetBoneIndex = function(_, name) return wing_bones[name] and 1 or -1 end,
    GetBoneTransform = function(_, name, space)
        assert(space == 2, 'component space expected')
        local b = wing_bones[name]
        return {Translation = {X = b[1], Y = b[2], Z = b[3]}}
    end,
}
object(6, 'BP_GR0001_tank_C', 100, 0, 0, {target = true, speed = 15})
object(7, 'BP_PlayerPlane_PP0004_f15c_C', 0, 0, 0, {hidden = true})
-- a lockable part of a boss: no speed of its own, moves with the aircraft
local part = object(8, 'BP_Boss_Part_Engine_C', 5000, 0, 5000, {target = true, speed = 0})
FindAllOf = function() return objects end

local packets, boss = {}, {}
local hooks = {}
RegisterHook = function(name, fn) hooks[name] = fn end
local contacts = dofile('Payload/Game/Binaries/Win64/UE4SS/Mods/AC8MouseAim/Scripts/contacts.lua')
contacts.init(function(text)
    if text:find('"type":"boss"') then boss[#boss + 1] = text else packets[#packets + 1] = text end
    return 1
end)
-- the scan runs only once a packet has gone out (telemetry on): one sample first
contacts.update(pawn, 0.5)
packets, boss = {}, {}
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
-- bosses alone, LADON with its wings from the skeleton (sweep 66.0, fold 35.4 on both sides);
-- a boss without that skeleton reports -1
assert(boss[1], 'no boss packet')
assert(boss[1]:find('%[9,"BP_OP0045_ladn_CP_C",20000.00,0.00,9000.00,[^%]]*,65%.96,35%.36,65%.96,35%.36,%d%]'), 'LADON wings: ' .. boss[1])
assert(boss[1]:find('"BP_Boss_Something_C"[^%]]*,%-1%.00,%-1%.00,%-1%.00,%-1%.00,%d%]'), 'boss without wings: ' .. boss[1])
assert(not boss[1]:find('mr2k'), 'only bosses, escorts and elites in a boss packet')
assert(boss[1]:find('BP_OP1020_m29a_CP_Shadow_C'), 'a Shadow (elite) not sampled fast')
assert(boss[1]:find('BP_OP1016_x40a_CP_Tewy_C'), 'Moon 11 not sampled as a boss')
assert(not boss[1]:find('x40a_RG_CP'), 'a boss part sampled as a boss')
assert(boss[1]:find('%[12,"BP_OP0024_uavn_CP_C",[^%]]*,%-1%.00,%-1%.00,%-1%.00,%-1%.00,0%]'), 'escort drone missing: ' .. boss[1])
assert(boss[1]:find('%[9,"BP_OP0045_ladn_CP_C",[^%]]*,0%]'), 'targeted flag missing')
assert(first:find('"BP_OP1017_mr2k_CP_C",900.0,0.0,3000.0,12.50,10.00,%-30.00'), 'lower-case pitch not read')
-- a LADON (not a player target itself) is tracked first despite being farthest
assert(first:find('ladn_CP_C') < first:find('mr2k'), 'LADON not ranked as an elite')
-- the part moves 400 m/s with its aircraft: picked up at the next scan (3 s on)
part.x = part.x + 1200
contacts.update(pawn, 4.1)
assert(packets[2]:find('BP_Boss_Part_Engine_C'), 'moving part not tracked')
packets = {packets[1]}
-- an address reused for a new object: the class is read again (no stale name)
local reused = object(10, 'BP_VE002_aegs_mk41_CP_Ally_C', 7000, 0, 0, {speed = 0})
contacts.update(pawn, 7.2)
reused.class = 'BP_OP0045_ladn_Engine_L_Base_C'; reused.z = 6000; reused.target = true
pawn.TargetSelectionComponent = {GetSelectedTarget = function() return reused end}
contacts.update(pawn, 7.4)
assert(packets[#packets]:find('ladn_Engine_L'), 'stale class for a reused address')
pawn.TargetSelectionComponent = {GetSelectedTarget = function() return nil end}
packets = {packets[1]}
-- shot down: hidden, then reported once as gone
enemy.hidden = true
contacts.update(pawn, 7.6)
assert(packets[2]:find('"gone":%[2%]'), 'gone not reported')
contacts.update(pawn, 7.8)
assert(not packets[3]:find('"gone"'), 'gone reported twice')
-- the game's targeting lists: the Shadow lockable, LADON a candidate not lockable now, the
-- wingman in neither (allies cannot be locked); the first read is logged beside the script
local function tarray(list) return {ForEach = function(self, fn) for i, v in ipairs(list) do fn(i, {get = function() return v end}) end end} end
local logged = {}
local real_open = io.open
io.open = function(path, mode) logged[#logged + 1] = path; return {write = function() end, close = function() end} end
pawn.TargetSelectionComponent = {GetSelectedTarget = function() return nil end,
    TargetCandidates = tarray({objects[3]}), UnTargetableCandidates = tarray({ladon})}
packets = {}
contacts.update(pawn, 8.0)
io.open = real_open
local list = packets[1]:match('"c":%[(.*)%]')
assert(list:find('%[3,"BP_OP1020_m29a_CP_Shadow_C",[^%]]*,2,%-?%d+%]'), 'lockable target not marked 2: ' .. list)
assert(list:find('%[9,"BP_OP0045_ladn_CP_C",[^%]]*,1,%-?%d+%]'), 'untargetable candidate not marked 1')
assert(list:find('%[4,"BP_WOP1004_f15c_C",[^%]]*,0,%-?%d+%]'), 'ally not marked 0')
assert(#logged == 1 and logged[1]:find('target%-candidates%-'), 'candidate lists not logged once')
pawn.TargetSelectionComponent = {GetSelectedTarget = function() return nil end}
-- a Tu-95's wing is not tracked unless selected
object(500, 'BP_OP0011_tu95_CP_LeftChildWing_C', 2500, 0, 3000, {target = true})
-- a huge furball: 800 more aircraft, sent in packets the native send accepts (< 60000 bytes)
for i = 1, 800 do object(1000 + i, 'BP_OP1001_f16c_CP_C', 2000 + i * 10, 500, 3000, {target = true}) end
packets = {}
contacts.update(pawn, 10.5)
local more, aircraft = 0, 0
for _, text in ipairs(packets) do
    assert(#text < 60000, 'packet too big: ' .. #text)
    if text:find('"type":"contacts_more"') then more = more + 1 end
    for _ in text:gmatch('BP_OP1001_f16c_CP_C') do aircraft = aircraft + 1 end
end
assert(packets[1]:find('"type":"contacts"'), 'first packet is not the contacts packet')
assert(more >= 2, 'a big sample not split: ' .. more)
assert(aircraft >= 800, 'aircraft lost when split: ' .. aircraft)
for _, text in ipairs(packets) do assert(not text:gsub('"classes":%b{}', ''):find('LeftChildWing'), 'a part tracked when not selected') end
-- damage and gun hits as the game reports them, with the victim's health after
assert(hooks['/Script/Live.LiveAIGameObject:OnLiveDamageTakenBP'] and hooks['/Script/Live.LiveGameObject:OnHitByGun'], 'hit hooks not registered')
enemy.HealthInternal = 40
local function param(v) return {get = function() return v end} end
packets = {}
hooks['/Script/Live.LiveAIGameObject:OnLiveDamageTakenBP'](param(enemy), param(15), param(pawn))
hooks['/Script/Live.LiveGameObject:OnHitByGun'](param(enemy), param(pawn))
assert(packets[1] and packets[1]:find('"type":"hit"') and packets[1]:find('"kind":"damage"') and packets[1]:find('"d":15%.00')
       and packets[1]:find('"v":2,') and packets[1]:find('"a":1,') and packets[1]:find('"h":40'), 'damage not reported: ' .. tostring(packets[1]))
assert(packets[2] and packets[2]:find('"kind":"gun"'), 'gun hit not reported: ' .. tostring(packets[2]))
print('Lua contacts checks passed')
