-- Other aircraft for the telemetry stream (dev\Telemetry.cmd): position, attitude and the
-- game's own speed of each aircraft at ~10 Hz, the selected target, and the player's own
-- game speed and aircraft class. Read-only. Does nothing while telemetry is off (the send
-- returns 0). Enemies are tracked first (elites always), then the nearest others; an
-- aircraft that disappears (destroyed, or removed by the game) is reported once in "gone".
local M={}
local send
local next_scan,next_sample,backoff_until=0,0,0
local class_of={}        -- address -> short class name (classes never change)
local tracked={}         -- objects sampled between scans
local classes_seen={}    -- class name -> count, last scan
local SCAN_SECONDS,SAMPLE_SECONDS,MAX_TRACKED=3,0.1,32

function M.init(send_function) send=send_function end

local function short_class(obj)
    local name='?'
    pcall(function() name=obj:GetClass():GetFName():ToString() end)
    return name
end
local function vec(v,key) local ok,x=pcall(function() return v[key] end); return ok and tonumber(x) or 0 end
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
-- Never aircraft: ground units, ships, props, training dummies, cutscene and minigame actors.
local function surface_class(name)
    return name:find('^BP_GR') or name:find('^BP_VE') or name:find('^BP_BPD') or name:find('^BP_AI_Dummy')
        or name:find('^BP_CIN') or name:find('MiniGame') or name:find('Minigame')
end
-- Named aces, Shadow squadrons and bosses (class names, logged: BP_OP1020_m29a_CP_Shadow_C,
-- BP_OP1007_f18c_CP_Named_C, LADON BP_OP0045_ladn_CP_C and its parts BP_OP0045_ladn_*): tracked
-- before anything else (a LADON battle brings a swarm of 140 drones).
local function elite_class(name)
    local n=name:lower()
    return n:find('shadow') or n:find('named') or n:find('boss') or n:find('ladon') or n:find('_ladn')
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

local self_id,self_class=nil,'?'
function M.update(pawn,game_time)
    if not send or game_time<0 or game_time<next_sample or game_time<backoff_until then return end
    next_sample=game_time+SAMPLE_SECONDS
    local new_scan=game_time>=next_scan
    if new_scan then
        next_scan=game_time+SCAN_SECONDS
        local ok=pcall(scan,pawn,game_time)
        if not ok then tracked={} end
    end
    local id=0
    pcall(function() id=pawn:GetAddress() end)
    if id~=self_id then self_id=id; self_class=short_class(pawn) end
    local selected=0
    pcall(function()
        local target=pawn.TargetSelectionComponent:GetSelectedTarget()
        if target and target:IsValid() then
            selected=target:GetAddress()
            if not class_of[selected] then class_of[selected]=short_class(target) end
            local listed=false
            for _,c in ipairs(tracked) do if c.id==selected then listed=true end end
            if not listed then tracked[#tracked+1]={obj=target,id=selected,cls=class_of[selected],enemy=is_enemy(target,class_of[selected]),d=0} end
        end
    end)
    local my_speed=-1
    pcall(function() my_speed=tonumber(pawn:GetSpeedMps()) or -1 end)
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
            return string.format('[%.0f,%s,%.1f,%.1f,%.1f,%.2f,%.2f,%.2f,%.1f,%d,%d]',
                c.id,json_string(c.cls),x,y,z,vec(r,'Pitch'),vec(r,'Yaw'),vec(r,'Roll'),speed,hidden,c.enemy and 1 or 0)
        end)
        if ok and entry then parts[#parts+1]=entry end
    end
    local text=string.format('{"type":"contacts","gt":%.4f,"selected":%.0f,"me":%.1f,"self":%s,"c":[%s]',
        game_time,selected,my_speed,json_string(self_class),table.concat(parts,','))
    if #gone>0 then text=text..',"gone":['..table.concat(gone,',')..']' end
    if new_scan then
        local names={}
        for name,count in pairs(classes_seen) do names[#names+1]=json_string(name)..':'..count end
        text=text..',"classes":{'..table.concat(names,',')..'}'
    end
    text=text..'}'
    local ok,sent=pcall(send,text)
    if not ok or sent~=1 then backoff_until=game_time+5 end
end

function M.reset() tracked={}; class_of={}; hostile_class={}; last_seen={}; next_scan=0; next_sample=0; self_id=nil end
return M
