-- No key binding guesses or global object searches. Focus: 0.2.24 capture;
-- ImpactCamera activation: 0.2.27 capture at 5.801s / 12.489s.
local M={}
local owner,focus,impact,gameplay
local last_active=false
local release_at=0
local failed=false
local manual_gaze=false
local hold_threshold=0.35 -- Mod debounce; not claimed to be AC's internal threshold.
function M.update(pawn,frame_time)
    if owner~=pawn:GetAddress() then
        owner=pawn:GetAddress(); focus=nil; impact=nil; last_active=false; release_at=0; failed=false; manual_gaze=false
    end
    if failed then return false end
    local ok,result=pcall(function()
        if not focus or not focus:IsValid() then focus=pawn.CameraViewComponent.CachedFocusTarget end
        if not impact or not impact:IsValid() then impact=pawn.ImpactCamera end
        local cinematic=impact and impact:IsValid() and impact.bIsActive==true
        if not frame_time and (not gameplay or not gameplay:IsValid()) then gameplay=StaticFindObject('/Script/Engine.Default__GameplayStatics') end
        local valid_focus=focus and focus:IsValid()
        local event=valid_focus and focus.ProcessingEventFocusTarget:Get()
        -- CandidateEventFocusTarget and FocusTarget alone are NOT activation signals.
        local pressed=valid_focus and focus.bFocusInputPrevPressed==true
        local held=valid_focus and (tonumber(focus.FocusInputHoldDuration) or 0) or 0
        if not pressed then manual_gaze=false
        elseif held>=hold_threshold then manual_gaze=true end
        local forced=valid_focus and focus.bForceInput==true
        local event_active=event and event:IsValid()
        local now=frame_time or gameplay:GetRealTimeSeconds(pawn)
        if manual_gaze or forced or event_active or cinematic then release_at=now+0.25 end
        local active=manual_gaze or forced or event_active or cinematic or now<release_at
        if active~=last_active then
            print(string.format('[AC8MouseAim] Gaze camera %s (longHold=%s held=%.3f forced=%s event=%s cinematic=%s)\n',
                active and 'yield' or 'resume',tostring(manual_gaze),held,tostring(forced),tostring(event_active==true),tostring(cinematic==true)))
            last_active=active
        end
        return active
    end)
    if not ok then
        failed=true
        print('[AC8MouseAim] Gaze detection unavailable: '..tostring(result)..'; F8 remains available.\n')
        return false
    end
    return result==true
end
-- Scripted scenes (the mission-end replay kept the aim circle on screen). AC8's replay
-- does not use Unreal's cinematic mode or ignored input (all stayed false in a logged
-- replay); it drives the player plane with replay cameras, and the plane's
-- OldReplayCameraComponent is set during the replay and empty in flight (F6 probe
-- captures 2026-10-04: replay vs live flight). The standard flags are kept as a backstop.
local scene_owner,scene_last,scene_failed
function M.scene(pawn,controller)
    if scene_failed then return false end
    if scene_owner~=pawn:GetAddress() then scene_owner=pawn:GetAddress(); scene_last=nil end
    local s={}
    pcall(function() s.cinematic=controller.bCinematicMode==true end)
    pcall(function() s.look=controller:IsLookInputIgnored()==true end)
    pcall(function() s.move=controller:IsMoveInputIgnored()==true end)
    pcall(function()
        local camera=pawn.OldReplayCameraComponent
        s.replay=camera~=nil and camera:IsValid()
    end)
    pcall(function()
        local target=controller:GetViewTarget()
        s.view=target~=nil and target:IsValid() and target:GetAddress()==pawn:GetAddress()
    end)
    local off=s.replay==true or s.cinematic==true or s.look==true or s.move==true
    local key=string.format('replay=%s cinematic=%s lookIgnored=%s moveIgnored=%s viewTargetIsPlane=%s',
        tostring(s.replay),tostring(s.cinematic),tostring(s.look),tostring(s.move),tostring(s.view))
    if key~=scene_last then
        print('[AC8MouseAim] Scene '..(off and 'scripted (yield)' or 'player')..': '..key..string.char(10))
        scene_last=key
    end
    return off
end
return M
