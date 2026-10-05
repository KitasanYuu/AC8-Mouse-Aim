-- Other aircraft for the telemetry stream (dev\Telemetry.cmd): position, attitude and the
-- game's own speed of each aircraft at ~10 Hz, the selected target, and the player's own
-- game speed and aircraft class. Read-only. Does nothing while telemetry is off (the send
-- returns 0). Enemies are tracked first (elites always), then the nearest others; an
-- aircraft that disappears (destroyed, or removed by the game) is reported once in "gone".
-- Bosses (LADON, Moon 11), their escorts (Moon 11's shield drones) and elites (Shadow, Named;
-- at most MAX_FAST of all these) are also sent on their own at ~30 Hz ("boss" packets), with the sweep and fold of LADON's variable wings read from
-- the skeleton and whether each is the player's current target.
local M={}
local send
local directory=(debug.getinfo(1,'S').source:sub(2):match('^(.*[/\\])')) or ''
local next_scan,next_sample,backoff_until=0,0,0
-- address -> short class name, re-read at every scan: the game reuses an address for a new
-- object (logged: LADON parts recorded under the names of destroyed allied ship parts)
local class_of={}
local tracked={}         -- objects sampled between scans
local classes_seen={}    -- class name -> count, last scan
-- Up to 1000 aircraft (the big furballs: a Shadow-boss mission opens with more than anyone
-- counted; 32 dropped the far ones). A sample goes out in packets of PER_PACKET aircraft: the
-- send takes under 60000 bytes. The first packet is "contacts" (time, selection, the class
-- census), the rest "contacts_more" with the same time, to be merged into it.
local SCAN_SECONDS,SAMPLE_SECONDS,MAX_TRACKED,PER_PACKET=3,0.1,1000,350
local BOSS_SECONDS,MAX_FAST=1/30,12
local next_boss=0

function M.init(send_function) send=send_function end
-- Telemetry is on: the last packet went out (the send returns 0 while it is off).
local sending=false
function M.active() return sending end

local function short_class(obj)
    local name='?'
    pcall(function() name=obj:GetClass():GetFName():ToString() end)
    return name
end
local function vec(v,key) local ok,x=pcall(function() return v[key] end); return ok and tonumber(x) or 0 end
-- AC8's rotator spells pitch in lower case (pitch/Yaw/Roll); the engine spelling as a fallback.
local function rot_pitch(r) local ok,x=pcall(function() return r.pitch end); x=ok and tonumber(x) or nil; return x or vec(r,'Pitch') end
local function location(obj)
    local p=obj:K2_GetActorLocation()
    return vec(p,'X')/100,vec(p,'Y')/100,vec(p,'Z')/100
end
-- Air units by the game's class naming (logged): BP_OP<number>_* other aircraft and
-- helicopters (f15c, su25, mi24, f18c ...), BP_WOP<number>_* (wingmen, presumably) and
-- player-type planes. Ground units are BP_GR*, ships BP_VE*, props BP_BPD*.
local function aircraft_class(name)
    return name:find('^BP_W?OP%d') or name:find('PlayerPlane') or name:lower():find('aircraft')
end
-- A lockable part of a bigger aircraft (Tu-95 wings and gun turret, LADON engines and hatches,
-- Moon 11's rail guns and laser pod): at its aircraft's position, so tracked only while it is
-- the player's selection (which says what is being attacked). 85 Tu-95s brought 255 parts.
local function part_class(name)
    return name:find('ChildWing') or name:find('_Base_C$') or name:find('_gsh%d') or name:find('LaserPod')
        or name:find('_x40a_RG')
end
-- Never aircraft: ground units, ships, props, training dummies, cutscene and minigame actors.
local function surface_class(name)
    return name:find('^BP_GR') or name:find('^BP_VE') or name:find('^BP_BPD') or name:find('^BP_AI_Dummy')
        or name:find('^BP_CIN') or name:find('MiniGame') or name:find('Minigame')
end
-- Named aces, Shadow squadrons and bosses (class names, logged: BP_OP1020_m29a_CP_Shadow_C,
-- BP_OP1007_f18c_CP_Named_C, LADON BP_OP0045_ladn_CP_C and its parts BP_OP0045_ladn_*, Moon 11
-- BP_OP1016_x40a_CP_Tewy_C): tracked before anything else (a LADON battle brings a swarm of
-- 140 drones).
local function elite_class(name)
    local n=name:lower()
    return n:find('shadow') or n:find('named') or n:find('boss') or n:find('ladon') or n:find('_ladn') or n:find('tewy')
        or n:find('toplak')   -- the Shadow squadron's leader (BP_OP1021_su57_CP_Toplak_C)
end
-- Friend or foe by the game itself: an object the player is meant to attack. The flag
-- comes and goes for an enemy (logged), so a class flagged once stays hostile.
local hostile_class={}
local function is_enemy(obj,cls)
    local ok,value=pcall(function() return obj:IsPlayerTarget() end)
    if ok and value==true and cls then hostile_class[cls]=true end
    return (ok and value==true) or (cls~=nil and hostile_class[cls]==true)
end
-- Still in the air: valid, not hidden, not parked at the origin (pooled planes wait
-- hidden or at the origin, which is also where a destroyed one goes).
local function flying(obj)
    local ok,valid=pcall(function() return obj:IsValid() end)
    if not ok or not valid then return false end
    local hidden=false
    pcall(function() hidden=obj.bHidden==true end)
    if hidden then return false end
    local ok2,x,y,z=pcall(location,obj)
    return ok2 and math.abs(x)+math.abs(y)+math.abs(z)>1
end

-- Where each candidate of an unrecognized class was at the last scan: parts attached to a
-- big aircraft (a boss's engines, say) may report no speed of their own but move with it.
local last_seen={}
local function moving_fast(obj,id,game_time)
    local speed=0
    pcall(function() speed=tonumber(obj:GetSpeedMps()) or 0 end)
    local ok,x,y,z=pcall(location,obj)
    if not ok then return speed>60 end
    local prev=last_seen[id]
    last_seen[id]={x,y,z,game_time}
    if prev and game_time>prev[4] then
        local d=math.sqrt((x-prev[1])^2+(y-prev[2])^2+(z-prev[3])^2)
        speed=math.max(speed,d/(game_time-prev[4]))
    end
    return speed>60
end

local function scan(pawn,game_time)
    local me=pawn:GetAddress()
    local px,py,pz=location(pawn)
    local found={}
    classes_seen={}
    class_of={}
    local all=FindAllOf('LiveGameObject')
    if not all then tracked={}; return end
    for _,obj in ipairs(all) do
        if obj:IsValid() then
            local id=obj:GetAddress()
            if id~=me then
                local cls=class_of[id]
                if not cls then cls=short_class(obj); class_of[id]=cls end
                classes_seen[cls]=(classes_seen[cls] or 0)+1
                local air=aircraft_class(cls)
                if not air and not surface_class(cls) then
                    -- A target of another kind moving at aircraft speed (a boss with its own
                    -- class name, or a lockable part of one): recorded as an aircraft.
                    air=is_enemy(obj,cls) and moving_fast(obj,id,game_time)
                end
                if air and part_class(cls) then air=false end   -- parts: only as the selection
                if air and flying(obj) then
                    local x,y,z=location(obj)
                    found[#found+1]={obj=obj,id=id,cls=cls,enemy=is_enemy(obj,cls),d=(x-px)^2+(y-py)^2+(z-pz)^2}
                end
            end
        end
    end
    -- elites, then enemies, then the rest, nearest first within each
    local function rank(c) return elite_class(c.cls) and 0 or c.enemy and 1 or 2 end
    table.sort(found,function(a,b)
        local ra,rb=rank(a),rank(b)
        if ra~=rb then return ra<rb end
        return a.d<b.d
    end)
    local before={}
    for _,c in ipairs(tracked) do before[c.id]=c end
    tracked={}
    for i=1,math.min(#found,MAX_TRACKED) do tracked[i]=found[i]; before[found[i].id]=nil end
    -- still-listed aircraft that are no longer flying were destroyed or removed: they are
    -- reported by the next sample (dropped ones that still fly are simply not sampled)
    for _,c in pairs(before) do
        if not flying(c.obj) then tracked[#tracked+1]=c end
    end
end

local function json_string(s) return '"'..tostring(s):gsub('[%c"\\]','')..'"' end

-- The game's own targeting: the player's LiveTargetSelectionComponent keeps TargetCandidates
-- (lockable now) and UnTargetableCandidates (candidates that cannot be locked now). Allies are
-- in neither (they cannot be locked). Read each sample: address -> 2 lockable, 1 not now.
-- The element type is not known yet: the first read writes what it saw beside the script.
local candidates_logged=false
local function element_address(elem)
    local obj=elem
    pcall(function() if elem.get then obj=elem:get() end end)
    local ok,id=pcall(function() return obj:GetAddress() end)
    if ok and id then return id end
    for _,field in ipairs({'Target','Actor','Object','TargetObject','Candidate','Owner'}) do
        local ok2,id2=pcall(function()
            local v=obj[field]; if v and v.get then v=v:get() end
            return v:GetAddress()
        end)
        if ok2 and id2 then return id2 end
    end
    return nil
end
local function read_candidates(pawn)
    local out,log,readable,total={},{},false,0
    local comp
    pcall(function() comp=pawn.TargetSelectionComponent end)
    if not comp then return out end
    for _,list in ipairs({{'TargetCandidates',2},{'UnTargetableCandidates',1}}) do
        local arr
        pcall(function() arr=comp[list[1]] end)
        local n=0
        if arr then
            pcall(function()
                arr:ForEach(function(_,elem)
                    n=n+1
                    local id=element_address(elem)
                    if id and (out[id] or 0)<list[2] then out[id]=list[2] end
                    if not candidates_logged and n<=3 then
                        local inner=elem
                        pcall(function() if elem.get then inner=elem:get() end end)
                        log[#log+1]=string.format('%s[%d]: element %s, value %s, address %s',
                            list[1],n,type(elem),tostring(inner),tostring(id))
                    end
                end)
            end)
        end
        if arr then readable=true end
        total=total+n
        log[#log+1]=list[1]..' count '..n..(arr and '' or ' (not readable)')
    end
    if not candidates_logged and readable and total>0 then   -- with something in them
        candidates_logged=true
        pcall(function()
            local f=io.open(directory..'target-candidates-'..os.date('%Y%m%d-%H%M%S')..'.txt','w')
            if f then f:write(table.concat(log,'\n'),'\n'); f:close() end
        end)
    end
    return out
end

-- Bosses: the aircraft itself (LADON BP_OP0045_ladn_CP_C, Moon 11 BP_OP1016_x40a_CP_Tewy_C),
-- not their lockable parts (BP_OP0045_ladn_Engine_*, BP_OP1016_x40a_RG_CP_C ...).
local function boss_class(name)
    local n=name:lower()
    return n:find('ladn_cp') or n:find('_cp_tewy') or (n:find('boss') and not n:find('part'))
end
-- A boss's escorts, sampled with it: Moon 11's drones (BP_OP0024_uavn_CP_C), which shield her
-- at first (shots do not reach her) and attack the player later.
local function escort_class(name)
    return name:lower():find('uavn_cp')
end
-- The variable wings, from the skeleton (bones logged by the mesh probe, component space,
-- X forward): sweep is the angle of the pivot -> outer joint line behind straight out (0
-- spread, ~66 swept), fold the angle the outer chain hangs below level. -1 when unreadable.
local WING={L={'BN_L_VariableWing_01','BN_L_VariableWing_02','BN_L_VariableWing_04'},
            R={'BN_R_VariableWing_01','BN_R_VariableWing_02','BN_R_VariableWing_04'}}
local skeleton={}   -- actor address -> {component, bone names} or false; cleared at each scan
local function bone_names()
    local out={}
    for side,list in pairs(WING) do
        out[side]={}
        for i,name in ipairs(list) do out[side][i]=FName(name) end
    end
    return out
end
local function find_skeleton(obj)
    local comp
    pcall(function() comp=obj.Mesh end)
    local names=bone_names()
    local function has_wing(c)
        local ok,index=pcall(function() return c:GetBoneIndex(names.L[1]) end)
        return ok and tonumber(index) and tonumber(index)>=0
    end
    if comp and has_wing(comp) then return {comp=comp,names=names} end
    local cls=StaticFindObject('/Script/Engine.SkeletalMeshComponent')
    local list=obj:K2_GetComponentsByClass(cls)
    for _,c in ipairs(list or {}) do
        local item=c
        pcall(function() if item.get then item=item:get() end end)
        if has_wing(item) then return {comp=item,names=names} end
    end
    return false
end
local function bone_at(comp,name)
    local t=comp:GetBoneTransform(name,2)   -- RTS_Component
    local p=t.Translation
    return vec(p,'X')/100,vec(p,'Y')/100,vec(p,'Z')/100
end
local function wing_angles(comp,names)
    local x1,y1,z1=bone_at(comp,names[1])
    local x2,y2,z2=bone_at(comp,names[2])
    local x4,y4,z4=bone_at(comp,names[3])
    local sweep=math.deg(math.atan(-(x2-x1),math.abs(y2-y1)))
    local fold=math.deg(math.atan(-(z4-z2),math.sqrt((x4-x2)^2+(y4-y2)^2)))
    return sweep,fold
end
local function boss_wings(c)
    local sk=skeleton[c.id]
    if sk==nil then
        local ok,found=pcall(find_skeleton,c.obj)
        sk=ok and found or false
        skeleton[c.id]=sk
    end
    if not sk then return -1,-1,-1,-1 end
    local ok,ls,lf=pcall(wing_angles,sk.comp,sk.names.L)
    local ok2,rs,rf=pcall(wing_angles,sk.comp,sk.names.R)
    return ok and ls or -1,ok and lf or -1,ok2 and rs or -1,ok2 and rf or -1
end
-- Bosses and escorts alone, at BOSS_SECONDS: position, attitude, speed, both wings (-1 when the
-- aircraft has none) and whether it is the player's current target (IsPlayerTarget: it follows
-- the selection, logged in the Moon 11 battle; it does not say whether shots reach it).
local function send_bosses(game_time)
    local parts={}
    for _,c in ipairs(tracked) do
        if #parts<MAX_FAST and (boss_class(c.cls) or escort_class(c.cls) or elite_class(c.cls)) then
            local ok,entry=pcall(function()
                if not flying(c.obj) then return nil end
                local x,y,z=location(c.obj)
                local r=c.obj:K2_GetActorRotation()
                local speed=-1
                pcall(function() speed=tonumber(c.obj:GetSpeedMps()) or -1 end)
                local ls,lf,rs,rf=-1,-1,-1,-1
                if boss_class(c.cls) then ls,lf,rs,rf=boss_wings(c) end
                local targeted=0
                pcall(function() if c.obj:IsPlayerTarget()==true then targeted=1 end end)
                return string.format('[%.0f,%s,%.2f,%.2f,%.2f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%.2f,%.2f,%d]',
                    c.id,json_string(c.cls),x,y,z,rot_pitch(r),vec(r,'Yaw'),vec(r,'Roll'),speed,ls,lf,rs,rf,targeted)
            end)
            if ok and entry then parts[#parts+1]=entry end
        end
    end
    if #parts==0 then return end
    local ok,sent=pcall(send,string.format('{"type":"boss","gt":%.4f,"b":[%s]}',game_time,table.concat(parts,',')))
    if not ok or sent~=1 then backoff_until=game_time+5 end
end

local self_id,self_class=nil,'?'
function M.update(pawn,game_time)
    if not send or game_time<0 or game_time<backoff_until then return end
    local boss_due=game_time>=next_boss
    if boss_due then next_boss=game_time+BOSS_SECONDS end
    if game_time<next_sample then
        if boss_due then send_bosses(game_time) end
        return
    end
    next_sample=game_time+SAMPLE_SECONDS
    local new_scan=game_time>=next_scan
    if new_scan then
        next_scan=game_time+SCAN_SECONDS
        local ok=pcall(scan,pawn,game_time)
        if not ok then tracked={} end
        skeleton={}   -- addresses are reused: look the wings up again
    end
    local id=0
    pcall(function() id=pawn:GetAddress() end)
    if id~=self_id then self_id=id; self_class=short_class(pawn) end
    local selected=0
    pcall(function()
        local target=pawn.TargetSelectionComponent:GetSelectedTarget()
        if target and target:IsValid() then
            selected=target:GetAddress()
            class_of[selected]=short_class(target)   -- the selection changes often: always current
            local listed=false
            for _,c in ipairs(tracked) do if c.id==selected then listed=true; c.cls=class_of[selected] end end
            if not listed then tracked[#tracked+1]={obj=target,id=selected,cls=class_of[selected],enemy=is_enemy(target,class_of[selected]),d=0} end
        end
    end)
    local my_speed=-1
    pcall(function() my_speed=tonumber(pawn:GetSpeedMps()) or -1 end)
    local lockable={}
    pcall(function() lockable=read_candidates(pawn) end)
    local parts,gone,keep={},{},{}
    for _,c in ipairs(tracked) do
        if flying(c.obj) then keep[#keep+1]=c else gone[#gone+1]=string.format('%.0f',c.id) end
    end
    tracked=keep
    for _,c in ipairs(tracked) do
        local ok,entry=pcall(function()
            if not c.obj:IsValid() then return nil end
            local x,y,z=location(c.obj)
            local r=c.obj:K2_GetActorRotation()
            local speed=-1
            pcall(function() speed=tonumber(c.obj:GetSpeedMps()) or -1 end)
            local hidden=0
            pcall(function() if c.obj.bHidden then hidden=1 end end)
            return string.format('[%.0f,%s,%.1f,%.1f,%.1f,%.2f,%.2f,%.2f,%.1f,%d,%d,%d]',
                c.id,json_string(c.cls),x,y,z,rot_pitch(r),vec(r,'Yaw'),vec(r,'Roll'),speed,hidden,c.enemy and 1 or 0,lockable[c.id] or 0)
        end)
        if ok and entry then parts[#parts+1]=entry end
    end
    local first=parts
    if #parts>PER_PACKET then first={} for i=1,PER_PACKET do first[i]=parts[i] end end
    local text=string.format('{"type":"contacts","gt":%.4f,"selected":%.0f,"me":%.1f,"self":%s,"c":[%s]',
        game_time,selected,my_speed,json_string(self_class),table.concat(first,','))
    if #gone>0 then text=text..',"gone":['..table.concat(gone,',')..']' end
    if new_scan then
        local names={}
        for name,count in pairs(classes_seen) do names[#names+1]=json_string(name)..':'..count end
        text=text..',"classes":{'..table.concat(names,',')..'}'
    end
    text=text..'}'
    local ok,sent=pcall(send,text)
    sending=ok and sent==1
    if not sending then backoff_until=game_time+5; return end
    for start=PER_PACKET+1,#parts,PER_PACKET do
        local chunk={}
        for i=start,math.min(start+PER_PACKET-1,#parts) do chunk[#chunk+1]=parts[i] end
        pcall(send,string.format('{"type":"contacts_more","gt":%.4f,"c":[%s]}',game_time,table.concat(chunk,',')))
    end
    if boss_due then send_bosses(game_time) end   -- after the scan, so a new boss is in the list
end

function M.reset() tracked={}; class_of={}; hostile_class={}; last_seen={}; skeleton={}; next_scan=0; next_sample=0; next_boss=0; self_id=nil end
return M
