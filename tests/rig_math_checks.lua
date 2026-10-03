local rig = dofile('Payload/Game/Binaries/Win64/UE4SS/Mods/AC8MouseAim/Scripts/rig_math.lua')
local function near(a,b,tolerance) assert(math.abs(a-b)<tolerance) end
local forward,up={X=1,Y=0,Z=0},{X=0,Y=0,Z=1}
for _=1,60 do
    forward,up=rig.step(forward,up,{X=0,Y=1,Z=0},1/60)
    near(forward.X^2+forward.Y^2+forward.Z^2,1,1e-5)
    near(up.X^2+up.Y^2+up.Z^2,1,1e-5)
    near(forward.X*up.X+forward.Y*up.Y+forward.Z*up.Z,0,1e-5)
end
assert(forward.Y>0.99)
print('Lua rig math checks passed')
