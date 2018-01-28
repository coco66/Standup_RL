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
require('Dynamixel')

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
local qLArm0 = Body.get_larm_position();
local qRArm0 = Body.get_rarm_position();
local qLLeg0 = Body.get_lleg_position();
local qRLeg0 = Body.get_rleg_position();

local qlarm, qrarm, qlleg, qrleg, att


local jointstep = 15*D2R

function move(q0,q,branch)
  local t = Body.get_time()
  local t_start = t
  local t_diff = t-t_start
  while t_diff <= 3 do

    local ph = math.max(0,math.min(1, t_diff/duration))
    if branch == 1 then 
      Body.set_larm_command((1-ph)*q0 + ph * q)
    elseif branch == 2 then 
      Body.set_rarm_command((1-ph)*q0 + ph * q) 
    elseif branch == 3 then 
      Body.set_lleg_command((1-ph)*q0 + ph * q) 
    else  
      Body.set_rleg_command((1-ph)*q0 + ph * q) 
    end
    t_diff = t - t_start 
  end
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

    local str=getch.get();
    if #str>0 then
      local byte = string.byte(str,1)
      
      if byte == string.byte("a") then
        Body.set_larm_hardness(0.1)
      elseif byte == string.byte("s") then
        Body.set_rarm_hardness(0.1)
      elseif byte == string.byte("d") then
        Body.set_lleg_hardness(0.1)
      elseif byte == string.byte("f") then
        Body.set_rleg_hardness(0.1)
      elseif byte == string.byte("z") then
        Body.set_larm_hardness(0.8)
      elseif byte == string.byte("x") then
        Body.set_rarm_hardness(0.8)
      elseif byte == string.byte("c") then
        Body.set_lleg_hardness(0.8)
      elseif byte == string.byte("v") then
        Body.set_rleg_hardness(0.8)
      elseif byte == string.byte("g") then  -- STAND UP        
        Motion.event("standupRL")
      elseif byte == string.byte("h") then  -- HAND DESIGN GETUP
        Motion.event("standupHS")
      elseif byte == string.byte("i") then
      Body.set_body_hardness(0.5)        
        qlarm = Body.get_larm_position()
        qrarm = Body.get_rarm_position()
        qlleg = Body.get_lleg_position()
        qrleg = Body.get_rleg_position()
        att = vector.new({Body.get_sensor_imuAngle(1),Body.get_sensor_imuAngle(2)})
      elseif byte == string.byte("l") then
        print(unpack(Body.get_lleg_position()))
        print(unpack(Body.get_rleg_position()))
      elseif byte == string.byte("p") then
        Body.set_body_hardness(0.5)
        Body.set_larm_command(vector.new(qlarm))
        Body.set_rarm_command(vector.new(qrarm)) 
        Body.set_lleg_command(vector.new(qlleg)) 
        Body.set_rleg_command(vector.new(qrleg))
        print("Attitude: ",unpack(att))
      end
    end
  --end
  if headsm_running == 0 then
    Body.set_head_command({headangle[1],headangle[2]-headPitchBias});
    --print("\nHead Yaw Pitch:", unpack(headangle*180/math.pi))
  end
end

function update()
  qLArm0 = Body.get_larm_position()
  qRARm0 = Body.get_rarm_position()
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
    Motion.update();
    Body.update()
    if headsm_running > 0 then
      HeadFSM.update();
      Team.update();
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

