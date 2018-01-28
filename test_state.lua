cwd = os.getenv('PWD')
require('init')
require('Config')

--Dynamixel = require'Dynamixel'
require('unix')
require('getch')
require('shm')
require('vector')
require('dcm')
require('mcm')
require('vcm')
require('wcm')
require('Speak')
require('Body')
require('Motion')
require('Team')
require('Kinematics')

smindex = 0;

Motion.entry();
darwin = false;
webots = false;

-- Enable OP specific 
if(Config.platform.name == 'OP') then
    darwin = true;
end
if string.find(cwd, "WebotsController") then
  cwd = cwd.."/../Player";
end
local optState = dofile(cwd.."/Motion/".."options.lua");
getch.enableblock(1);
unix.usleep(1E6*1.0);
Body.set_body_hardness(0);

--This is robot specific 
webots = false;
init = false;
calibrating = false;
ready = false;
if( webots or darwin) then
    ready = true;
end

--State variables
initToggle = true;
targetvel=vector.zeros(3);
headangle=vector.new({0,10*math.pi/180});
headsm_running=0; --head state machine
bodysm_running=0; --body state machine

local count = 0;
local ncount = 100;
local imagecount = 0;
local t0 = unix.time();
local tUpdate = t0;

-- Broadcast the images at a lower rate than other data
local broadcast_enable = 0;
local imageCount = 0;

-- set game state to ready to stop init particle filter during debugging
-- gcm.set_game_state(1);

--main loop
count = 0;
lcount = 0;
tUpdate = unix.time();

fsm.enable_walkkick = 0;
fsm.enable_sidekick = 0;
broadcast_enable = 0;
button_pressed = {0,0};

Body.set_body_hardness(0)
Body.set_larm_hardness(0)
Body.set_rarm_hardness(0)

D2R = math.pi/180
local jointstep = 15*D2R

function domotion(q0,q1,duration,t_start)
  Body.set_larm_hardness(0.8)
  Body.set_rarm_hardness(0.8)
  Body.set_lleg_hardness(0.8)
  Body.set_rleg_hardness(0.8)
  Body.set_head_hardness(0)
  local t = Body.get_time()
  local t_diff = t-t_start
  while t_diff <= duration do 
    local ph = math.max(0,math.min(1, t_diff/duration))
    Body.set_larm_command((1-ph)*vector.new(q0[1]) + ph *vector.new( q1[1]))
    Body.set_rarm_command((1-ph)*vector.new(q0[2]) + ph * vector.new(q1[2]) )
    Body.set_lleg_command((1-ph)*vector.new(q0[3]) + ph * vector.new(q1[3]) )
    Body.set_rleg_command((1-ph)*vector.new(q0[4]) + ph * vector.new(q1[4])  )
    t = Body.get_time()
    t_diff = t-t_start
  end
  print("Done")
end

function process_keyinput()

  headPitchBiasComp = mcm.get_walk_headPitchBiasComp();
  headPitchBias = mcm.get_headPitchBias()

  --Toggle body SM when button is pressed and then released
  if (Body.get_change_state() == 1) then -- A button is pressed
    button_pressed[1] = 1;
  else
    if button_pressed[1]==1 then -- Any button has not been pressed but marked as pressed
      if bodysm_running == 0 then 
        Motion.event("standup")
        Body.set_head_hardness(0.5)
        vcm.set_camera_learned_new_lut(1)
        headsm_running=1;
        bodysm_running=1;
        HeadFSM.sm:set_state('headReady')
        BodyFSM.sm:set_state('bodyWait')

      else -- When the Body State Machine is running but a button hasn't been pressed, stop the Head State Machine First
        headsm_running = 0;
        Body.set_head_hardness(0)
        Body.set_head_command({0,0})
        
        if walk.active then walk.stop() end
        bodysm_running = 0;
        Motion.event("sit")
      end
    end
    button_pressed[1]=0;
  end

  if (Body.get_change_role() ==1) then
    putton_pressed[2]=0;
  else
    if button_pressed[2]==1 then -- Disable broadcast switching
      if broadcast_enable == 0 then 
         broadcast_enable = 2;
         Speak.talk('enable broadcasting');
      else
        broadcast_enable = 0
        Speak.talk('disable broadcasting');
      end
    end
    button_pressed[2]=0;
  end

  --local str=getch.get();
  --if #str > 0 then
  --[[
  io.write("Enter \" ID \" if you want to give a number of ID to see\n")
  io.write("Enter \" single \" if you want to work with single key inputs")
  key = io.read();
  if key == "ID" then
    io.write("Enter a number of ID")
  elseif key == "single" then
    io.write("Enter a key")
    ]]
  local q0 = {}
  q0[1] = vector.new(Body.get_larm_position());
  q0[2] =  vector.new(Body.get_rarm_position());
  q0[3] =  vector.new(Body.get_lleg_position());
  q0[4] =  vector.new(Body.get_rleg_position());

  local str=getch.get();
  if #str>0 then
    local byte = string.byte(str,1)
    
    if byte <= string.byte("9") then
      local o = byte - 48
      local q1 = {}
      q1[1] = vector.slice(optState[o],1,3)
      q1[2] = vector.slice(optState[o],4,6)
      q1[3] = vector.slice(optState[o],7,12)
      q1[4] = vector.slice(optState[o],13,18)
      domotion(q0,q1,2, Body.get_time());
      print("current state ", o)
      print("Desired: ", optState[o][19],optState[o][20])
    elseif (byte > 96) and (byte < (string.byte("k"))) then
      local o = byte - 87 -- "a" -> 10   "o"->25
      local q1 = {}
      q1[1] = vector.slice(optState[o],1,3)
      q1[2] = vector.slice(optState[o],4,6)
      q1[3] = vector.slice(optState[o],7,12)
      q1[4] = vector.slice(optState[o],13,18)
      domotion(q0,q1,2, Body.get_time());
      print("Desired: ", optState[o][19],optState[o][20])
      print("current state ", o)
    elseif byte == string.byte("l") then
      Body.set_larm_hardness(0)
    elseif byte == string.byte("m") then
      Body.set_rarm_hardness(0)
    elseif byte == string.byte("n") then
      Body.set_lleg_hardness(0)
    elseif byte == string.byte("o") then
      Body.set_rleg_hardness(0)
    elseif byte == string.byte("p") then
      Body.set_larm_hardness(0.8)
    elseif byte == string.byte("q") then
      Body.set_rarm_hardness(0.8)
    elseif byte == string.byte("r") then
      Body.set_lleg_hardness(0.8)
    elseif byte == string.byte("s") then
      Body.set_rleg_hardness(0.8)
    elseif byte == string.byte("t") then 
      print(unpack(q0))
    elseif byte == string.byte("z") then
      print("Current: ", Body.get_sensor_imuAngle(1),Body.get_sensor_imuAngle(2))
    end
  end
  --end
end

function update()

  count = count + 1;
  wcm.set_robot_battery_level(Body.get_battery_level())
  --Set game state to SET to prevent particle resetting
  --gcm.set_game_state(1);
  --Body.set_body_velocity(1,0.001);
  --print(Body.get_sensor_position());
  if (not init) then
    if (calibrating) then
      if (Body.calibrate(count)) then
        Speak.talk('Calibration done')
        calibrating = false
        ready = true
      end
    elseif (ready) then
      package.path = cwd..'/BodyFSM/'..Config.fsm.body[smindex+1]..'/?.lua;'..package.path;
      package.path = cwd..'/HeadFSM/'..Config.fsm.head[smindex+1]..'/?.lua;'..package.path;
      require('BodyFSM')
      require('HeadFSM')
      BodyFSM.entry();
      HeadFSM.entry();
      Team.entry();
      init = true;
    else
      if (count % 20 == 0) then
        --start calbrating without waiting
        Speak.talk('Calibrating')
        calibrating = true;
      end
      if (count % 100 == 0 ) then
        initToggle = not initToggle;
        if (initToggle) then
          Body.set_indicator_state({1,1,1});
        else
          Body.set_indicator_state({0,0,0});
        end
      end
    end
  else
    -- Update state machines
    process_keyinput();
    --Motion.update();
    --Body.update()
    --Body.set_body_hardness(1)
  end

  local dcount = 50;
  if (count % 50 == 0 ) then
    tUpdate = unix.time();
    Body.set_indicator_batteryLevel(Body.get_battery_level());
  end

  lcount = lcount + 1;
  if ( count ~= lcount) then
    print('count: '..count)
    print('lcount: '..lcount)
    Speak.talk('missed cycle');
    lcount = count;
  end
end

if (darwin or newnao ) then
  local tDelay = 0.005 * 1E6;
  while 1 do
    process_keyinput();
    unix.usleep(tDelay);
    update();
  end
end

