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
require('gcm')
require('Team')
require('Kinematics')

--gcm.say_id()
smindex = 0;

Motion.entry();
darwin = false;
webots = false;

-- Enable OP specific 
if(Config.platform.name == 'OP') then
    darwin = true;
end

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

---[[ ? ]]-------------------------------------------------------------------------------------------------------------------------------
local motion_num = 0
local t_diff = 0 
local t_start = 0 
local t = 0
local duration = 0
local qLArm1, qRArm1, qLLeg1, qRLeg1
local cwd = unix.getcwd();
local optState =dofile(cwd.."/../Player/Motion/".."options.lua");
------------------------------------------------------------------------------------------------------------------------------------------
--State variables
initToggle = true;
targetvel=vector.zeros(3);
headangle=vector.new({0,10*math.pi/180});
headsm_running=0; --head state machine
bodysm_running=0; --body state machine

local count = 0;
local ncount = 100;
local t0 = unix.time();
local tUpdate = t0;

local broadcast_enable = 0;

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

  local str=getch.get();
  if #str>0 then
    local byte = string.byte(str,1)

    if (byte <= string.byte("9")) and (byte >= string.byte("1")) then
      num = tonumber(str)    
      t_diff = 0
      t_start = Body.get_time() 
      duration = 5
      --print("Press a number you want to check: ")
      --local str2 = getch.get();
      --local num = 21 --tonumber(str2)
      print("The desired Attitude is: ",optState[num][19],optState[num][20])
      Body.set_body_hardness(0.8)
      qLArm1 = vector.slice(cent[num],1,3)
      qRArm1 = vector.slice(cent[num],4,6)
      qLLeg1 = vector.slice(cent[num],7,12)
      qRLeg1 = vector.slice(cent[num],13,18)

    elseif byte == string.byte("0") then 
      print("Roll: ", Body.get_sensor_imuAngle(1))
      print("Pitch: ", Body.get_sensor_imuAngle(2))

    elseif byte == string.byte("a") then
      Body.set_larm_hardness(0)
    elseif byte == string.byte("s") then
      Body.set_rarm_hardness(0)
    elseif byte == string.byte("d") then
      Body.set_lleg_hardness(0)
    elseif byte == string.byte("f") then
      Body.set_rleg_hardness(0)
    elseif byte == string.byte("z") then
      Body.set_larm_hardness(0.8)
    elseif byte == string.byte("x") then
      Body.set_rarm_hardness(0.8)
    elseif byte == string.byte("c") then
      Body.set_lleg_hardness(0.8)
    elseif byte == string.byte("v") then
      Body.set_rleg_hardness(0.8)
    elseif byte == string.byte("p") then
      print("LA: ", vector.new(Body.get_larm_position()))
      print("RA: ", vector.new(Body.get_rarm_position()))
      print("LL: ", vector.new(Body.get_lleg_position()))
      print("RL: ", vector.new(Body.get_rleg_position()))
      print("Roll: ", Body.get_sensor_imuAngle(1))
      print("Pitch: ", Body.get_sensor_imuAngle(2))
    end
    if headsm_running == 0 then
      Body.set_head_command({headangle[1],headangle[2]-headPitchBias});
      print("\nHead Yaw Pitch:", unpack(headangle*180/math.pi))
    end
  end
end

function update()

  local qLArm0 = vector.new(Body.get_larm_position());
  local qRArm0 = vector.new(Body.get_rarm_position());
  local qLLeg0 = vector.new(Body.get_lleg_position());
  local qRLeg0 = vector.new(Body.get_rleg_position());
  
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
    while t_diff < duration do 

      t = Body.get_time()
      t_diff = t - t_start
      ph = math.max(0,math.min(1, t_diff/duration))
          
      Body.set_larm_command((1-ph)*qLArm0 + ph * qLArm1)
      Body.set_rarm_command((1-ph)*qRArm0 + ph * qRArm1) 
      Body.set_lleg_command((1-ph)*qLLeg0 + ph * qLLeg1) 
      Body.set_rleg_command((1-ph)*qRLeg0 + ph * qRLeg1)

    end

    if bodysm_running > 0 then
      BodyFSM.update();
    end
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

