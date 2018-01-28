-- Author : Heejin Jeong (University of Pennsylvania)
-- Start date : June 24th, 2015
-- Last modified : July 30th, 2016
-- Description: This file is for executing stand up motions learned by the Reinforcement Learning methods. This will follow the optimal policy written below.

module(..., package.seeall);
name = ...;

webots = false;
darwin = true;

require('Motion')
require('Body')
require('vector')
require('unix')
require('Kinematics')
require('Getup')
require('mcm')
require('Config')

local Getup = require'Getup'

local sqrt 	= math.sqrt
local sin 	= math.sin
local cos	= math.cos
local pi	= math.pi
local abs	= math.abs

local D2R	= pi/180
local init_local = 1
local duration = 1.5
local Rmax = 1
local Rsup = 1

-- [ Robot Specs ]-------------------------------------------------------------------------------
local joint_MAX ={ 	vector.new({175,175, 1})*D2R,  --larm	
                  	vector.new({175, -5, 1})*D2R, 	--rarm
                  	vector.new({ 45, 80, 20, 135, 72, 72})*D2R, --lleg
                  	vector.new({145, 80, 20, 135, 72, 72})*D2R --rleg
                  } 
local joint_MIN = { vector.new({-175,    5, -140})*D2R, --larm
                  	vector.new({-175, -175, -140})*D2R, --rarm
                  	vector.new({-145,  -80, -110, -9, -85, -72})*D2R,
                  	vector.new({-45,   -80, -110, -9, -85, -72})*D2R
                  }


---[ Variables ]---------------------------------------------------------------------------------
local qLArm0, qRArm0,qLLeg0,qRLeg0
local qLArm1, qRArm1,qLLeg1,qRLeg1
local prop = 1
local LS 
local stateO, state_pre, sym


local cwd = unix.getcwd();
if string.find(cwd, "WebotsController") then
  cwd = cwd.."/../Player";
end
local optState = dofile(cwd.."/Motion/".."stateSym.lua");--dofile(cwd.."/Motion/".."options.lua");
-- RL learned results
local opt28 = dofile(cwd.."/Motion/".."opt_policy28.lua")
local optLS = dofile(cwd.."/Motion/".."opt_policyLS.lua")
-- 07.30.2016
--local optLS2 = dofile(cwd.."/Motion/".."opt_policyLS2.lua")
--local opt282 = dofile(cwd.."/Motion/".."opt_policy282.lua")
local stateNum = #optState

function entry()
	--fid = io.open("opt2801.txt","w")
	--act = opt2
	fid = io.open("LSnew01.txt","w")
	act = optLS2
	LS = 1

	print("Motion SM:".._NAME.." entry")
	
  -- TIME
	t_entry = Body.get_time()
	t_start = t_entry


  gyros = 0
  gyros_pre = gyros
  angAcc = 0
  t_pre = t_start
	execution = 0
  
	stepO = 1
	stepO_pre = 0

  -- Get Data from the robot
	qLArm0 = vector.new(Body.get_larm_position())  
	qRArm0 = vector.new(Body.get_rarm_position())  
	qLLeg0 = vector.new(Body.get_lleg_position())  
	qRLeg0 = vector.new(Body.get_rleg_position()) 


	if webots then 
		qLArm1 = vector.new({1.2,	0.6,	-1.0})--vector.slice(optState[o],1,3)
	  	qRArm1 = vector.new({1.3,	-0.9,	-1.2}) --vector.slice(optState[o],4,6)
	  	qLLeg1 = vector.new({0,			 0,		-1,		1.0,	-0.6,	0}) --vector.slice(optState[o],7,12)
	  	qRLeg1 = vector.new({0,		-0.8,		-1,		1.0,	-0.8,		0})--vector.slice(optState[o],13,18)
	else 
		init_local = 0 
	  --quat = Body.get_sensor_quaternion()
		rpy = {Body.get_sensor_imuAngle(1),Body.get_sensor_imuAngle(2),Body.get_sensor_imuAngle(3)}
		pitch_pre = rpy[2]
		rawStateVec = util.concat(qLArm0, qRArm0)
		rawStateVec = util.concat(rawStateVec, qLLeg0)
		rawStateVec = util.concat(rawStateVec, qRLeg0)
		rawStateVec = util.concat(rawStateVec,vector.new({rpy[1],rpy[2]}))

		if LS == 1 then
			stateO, prop, sym = Getup.make_stateSYM(rawStateVec)
		else
			stateO = Getup.make_state(rawStateVec)
		end
		--minH, supStateVec = Getup.getSup(quat, qLArm0, qRArm0, qLLeg0, qRLeg0) -- ex) [1,0,0,1]^T
		--minH_pre = minH
		t_pre = t_start 
		angAcc = 0
		gyro_pre = vector.new(Body.get_sensor_imuGyrRPY())*D2R
		rpy_pre = rpy
	end
end

function update()
	local t = Body.get_time()
	local t_diff = t - t_start
 	--if tbreak == 0 then
	  if init_local == 0 then 
	  	if execution == 0 then
			  print("======step: ",stepO,"==========")
			  -- [[Middle Level Option]]--
			if stepO_pre ~= stepO then

				o = act[stateO]
				--o = optsymNo[stateO]
				stepO_pre = stepO

			end

			print("state ", stateO, "action ", o)
			local o_real

			if LS == 1  then
				if sym==0 and o > 28 then -- s, a sym
					if o < 39 then
						o_real = o-28
					else
						o_real = o-25
					end
					qRArm1 = vector.slice(optState[o_real],1,3)
					qLArm1 = vector.slice(optState[o_real],4,6)
					qRLeg1 = vector.slice(optState[o_real],7,12)
					qLLeg1 = vector.slice(optState[o_real],13,18)

					qLArm1[2] = -qLArm1[2]
				    qRArm1[2] = -qRArm1[2]
				    qLLeg1[1] = -qLLeg1[1]
				    qRLeg1[1] = -qRLeg1[1]
				    qLLeg1[2] = -qLLeg1[2]
				    qRLeg1[2] = -qRLeg1[2]
				    qLLeg1[6] = -qLLeg1[6]
				    qRLeg1[6] = -qRLeg1[6]

				elseif sym == 1 then 
					if o < 22 then -- s sym, and (s,a) pair, a should be converted to a sym 
						qRArm1 = vector.slice(optState[o],1,3)
						qLArm1 = vector.slice(optState[o],4,6)
						qRLeg1 = vector.slice(optState[o],7,12)
						qLLeg1 = vector.slice(optState[o],13,18)

						qLArm1[2] = -qLArm1[2]
					    qRArm1[2] = -qRArm1[2]
					    qLLeg1[1] = -qLLeg1[1]
					    qRLeg1[1] = -qRLeg1[1]
					    qLLeg1[2] = -qLLeg1[2]
					    qRLeg1[2] = -qRLeg1[2]
					    qLLeg1[6] = -qLLeg1[6]
					    qRLeg1[6] = -qRLeg1[6]

					elseif o > 28 then -- s sym, and (s, a sym) pair, a sym should be converted to a 
						if o < 39 then 
							o_real = o-28
						else 
							o_real = o-25
						end
						qLArm1 = vector.slice(optState[o_real],1,3)
						qRArm1 = vector.slice(optState[o_real],4,6)
						qLLeg1 = vector.slice(optState[o_real],7,12)
						qRLeg1 = vector.slice(optState[o_real],13,18)
					else
						qLArm1 = vector.slice(optState[o],1,3)
						qRArm1 = vector.slice(optState[o],4,6)
						qLLeg1 = vector.slice(optState[o],7,12)
						qRLeg1 = vector.slice(optState[o],13,18)
					end
				else
					qLArm1 = vector.slice(optState[o],1,3)
					qRArm1 = vector.slice(optState[o],4,6)
					qLLeg1 = vector.slice(optState[o],7,12)
					qRLeg1 = vector.slice(optState[o],13,18)
				end
			else

				qLArm1 = vector.slice(optState[o],1,3)
				qRArm1 = vector.slice(optState[o],4,6)
				qLLeg1 = vector.slice(optState[o],7,12)
				qRLeg1 = vector.slice(optState[o],13,18)
			end
			execution = 1		 	

	    else --- Action is Excuted! 
			  --print("execute?",t_diff/duration)
	      if t_diff <= duration then -- execution
	      local ph = math.max(0,math.min(1, t_diff/duration))
					
		  	Body.set_larm_command((1-ph)*qLArm0 + ph * qLArm1)
				Body.set_rarm_command((1-ph)*qRArm0 + ph * qRArm1) 
				Body.set_lleg_command((1-ph)*qLLeg0 + ph * qLLeg1) 
				Body.set_rleg_command((1-ph)*qRLeg0 + ph * qRLeg1)  
				
				gyro = vector.new(Body.get_sensor_imuGyrRPY())*D2R
				angAcc = angAcc + vector.norm((gyro - gyro_pre))/(t - t_pre)
				gyro_pre = gyro
				t_pre = t

				elseif t_diff > 1.5*duration then -- update
					--unix.usleep(1000000)
					qLArm0  = vector.new(Body.get_larm_position())  
				  qRArm0  = vector.new(Body.get_rarm_position())  
			  	qLLeg0  = vector.new(Body.get_lleg_position())  
				  qRLeg0  = vector.new(Body.get_rleg_position())

				  q = qLArm0
				  q = util.concat(q,qRArm0)
				  q = util.concat(q,qLLeg0)
				  q = util.concat(q,qRLeg0)

				  --quat = Body.get_sensor_quaternion()
				  rpy  = Body.get_sensor_imuAngle()				  
				  
				  --minH, supStateVec = Getup.getSup(quat, qLArm0, qRArm0, qLLeg0, qRLeg0, rpy[2]) -- ex) [1,0,0,1]^T
				  
				  rawStateVec = util.concat(qLArm0, qRArm0)
					rawStateVec = util.concat(rawStateVec, qLLeg0)
					rawStateVec = util.concat(rawStateVec, qRLeg0)
					rawStateVec = util.concat(rawStateVec,vector.new({rpy[1],rpy[2]}))

					state_pre = stateO
					o_pre = o
					if LS == 1 then
						stateO, prop, sym = Getup.make_stateSYM(rawStateVec)
					else
						stateO = Getup.make_state(rawStateVec)
					end
					print("next State: ",stateO, "executed action: ", o_pre) 
				  
				  --print("H ", abs(minH),"  ANG ACC ", angAcc)				  
				  print("RPY: ", unpack(rpy))--rpy[1]/D2R,rpy[2]/D2R,rpy[3]/D2R)
				  print("angAcc ", angAcc)
				  ----- WRITING -----------------------------------------------------------------------------------	
				  
				  fid:write(stepO,"\t",	angAcc,"\t")
				  for it=1,#rpy do
				  	fid:write(rpy[it],"\t")
				  end
				  for it=1,#q do
				  	fid:write(q[it],"\t")
				  end
				  fid:write(state_pre,"\t",o,"\n")
				
					fid:write(state_pre,"\t", o_pre, "\t", stateO,"\n")
					----- RESET -----------------------------------------------------------------------------------	
					--minH_pre = minH	
					stepO = stepO + 1
					t_start = t
					rpy_pre = rpy
					gyro_pre = vector.new(Body.get_sensor_imuGyrRPY())*D2R
					angAcc = 0
					t_pre = t
					--gyros = 0 
	        		execution = 0
	       			 --tbreak = 1
	        ----- TERMINATION -----------------------------------------------------------------------------------	
				  if (stateO == 29) and (abs(rpy[1])<0.3 and abs(rpy[2])<0.5) then
				  		print("DONE")
				  		io.close(fid)
				  		return'done'
				  end

	      else -- waiting 
	       	gyro = vector.new(Body.get_sensor_imuGyrRPY())*D2R
					angAcc = angAcc + vector.norm((gyro - gyro_pre))/(t - t_pre)
					gyro_pre = gyro
					t_pre = t
				end -- time duration ends

			end -- Action execution end
		else -- init == 1
			if t_diff <= duration then

		    local ph = math.max(0,math.min(1, t_diff/duration))

		  	Body.set_larm_command((1-ph)*qLArm0 + ph * qLArm1)
				Body.set_rarm_command((1-ph)*qRArm0 + ph * qRArm1) 
				Body.set_lleg_command((1-ph)*qLLeg0 + ph * qLLeg1) 
				Body.set_rleg_command((1-ph)*qRLeg0 + ph * qRLeg1) 
			
			elseif t_diff > 1.5*duration then --and hcm.get_state_proceed()==1 then
				qLArm0  = vector.new(Body.get_larm_position())  
			  qRArm0  = vector.new(Body.get_rarm_position())  
		  	qLLeg0  = vector.new(Body.get_lleg_position())  
			  qRLeg0  = vector.new(Body.get_rleg_position())
			  -- State Update
			  --quat = Body.get_sensor_quaternion()
			  rpy = {Body.get_sensor_imuAngle(1),Body.get_sensor_imuAngle(2),Body.get_sensor_imuAngle(3)}
			  --print("quat", unpack(quat))
				rawStateVec = util.concat(qLArm0, qRArm0)
					rawStateVec = util.concat(rawStateVec, qLLeg0)
					rawStateVec = util.concat(rawStateVec, qRLeg0)
					rawStateVec = util.concat(rawStateVec,vector.new({rpy[1],rpy[2]}))
				stateO, prop = Getup.make_state(rawStateVec)
				state_pre = stateNum
				o_pre = o
				--minH, supStateVec = Getup.getSup(quat, qLArm0, qRArm0, qLLeg0, qRLeg0,rpy[2]) -- ex) [1,0,0,1]^T
				--minH_pre = minH
				--print("INITIAL HEIGHT ", abs(minH))
				--tbreak = 1
				--unix.usleep(1000000)
				t_start = t
				t_pre = t
				angAcc = 0
				gyro_pre = vector.new(Body.get_sensor_imuGyrRPY())*D2R
				rpy_pre = rpy
				init_local = 0 
				execution = 0
			end
		end
end

function exit()

end


function joint_lim(q, mv)
	local lim = 0
	local i = 1
	while i < (#q+1) do
   -- print(mv,i,q[i],joint_MAX[mv][i],joint_MIN[mv][i])
   	if q[i] > math.pi then
   		q[i] = q[i] - 2*math.pi
    elseif q[i] < -math.pi then
    	q[i] = q[i] + 2*math.pi
    end

		if (q[i] > joint_MAX[mv][i]) or (q[i] < joint_MIN[mv][i]) then
			lim = 1
			print("joint limit")
			print("why?", q[i], joint_MAX[mv][i], joint_MIN[mv][i], i)
			break
		else
			i = i+1
		end 
	end
	if lim == 1 then
    print("joint limit?",i)
  end
  return lim
end


	































