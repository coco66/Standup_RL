-- Author : Heejin Jeong (University of Pennsylvania)
-- Start date : June 24th, 2015
-- Description: This file is for learning standup motions using Reinforcement Learning (MDP, not SMDP). This is the latest version among getup_RL*.lua and getupSim.lua.

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

local atan2 = math.atan2
local asin 	= math.asin
local acos 	= math.acos
local sqrt 	= math.sqrt
local sin 	= math.sin
local cos	= math.cos
local pi	= math.pi
local abs	= math.abs

local D2R	= pi/180
local init = 1
local duration = 3
local Rmax = 1
local Rsup = 1

local timeout = 30
local alpha = 0
local gamma = 1

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


local cwd = unix.getcwd();
local optState = dofile(cwd.."/../Player/Motion/".."options.lua");
local Qs = dofile(cwd.."/../Player/Motion/".."Qs.lua")
local stateNum = #optState
local N_Finit = 1
local N_Fend = 18
local N_Binit = 19
local N_Bend = 31

local visitinit = 0
local visitnum = visitinit*vector.ones(stateNum)
local Qgap = 0.07
local howlong = vector.zeros(stateNum)

local explore1 = {}
for i = N_Finit,N_Fend do
	explore1[i] = i
end
	explore1[N_Fend+1] = N_Bend+1
	explore1[N_Fend+2] = N_Bend+2
local explore2 = {}
for i =N_Binit,(N_Bend+2) do
	explore2[i-N_Fend] = i
end

local findit = vector.zeros(stateNum)
findit[N_Bend+1] = stateNum
findit[N_Bend+2] = stateNum

function entry()
	fid = io.open("test01.lua","w")

	print("Motion SM:".._NAME.." entry")

	Body.set_larm_hardness(0.2)
	Body.set_rarm_hardness(0.2)
	Body.set_lleg_hardness(0.2)
	Body.set_rleg_hardness(0.2)

	unix.usleep(100000)

	Body.set_larm_hardness(0.5)
	Body.set_rarm_hardness(0.5)
	Body.set_lleg_hardness(0.5)
	Body.set_rleg_hardness(0.5)

	unix.usleep(100000)

	Body.set_larm_hardness(0.7)
	Body.set_rarm_hardness(0.7)
	Body.set_lleg_hardness(0.7)
	Body.set_rleg_hardness(0.7)
	Body.set_head_hardness(0)
	-- TIME
	t_entry = Body.get_time()
	t_start = t_entry


	z = 1  
	gyros = 0
	gyros_pre = gyros
	angAcc = 0
	t_pre = t_start
	execution = 0
  
	stepO = 1
	stepO_pre = 0

	-- Initialize Q and visits
	Q = Qs.Q_raw-- constMatrix(qinit,stateNum, stateNum)
	visits = util.constMatrix(stateNum,stateNum,0)
	--Reward
	rwA = {}
	rwO = {}
-- Get Data from the robot
	qLArm0 = vector.new(Body.get_larm_position())  
	qRArm0 = vector.new(Body.get_rarm_position())  
	qLLeg0 = vector.new(Body.get_lleg_position())  
	qRLeg0 = vector.new(Body.get_rleg_position()) 

	o = 4
	print("Initial: ", o)
	qLArm1 = vector.slice(optState[o],1,3)
	qRArm1 = vector.slice(optState[o],4,6)
	qLLeg1 = vector.slice(optState[o],7,12)
qRLeg1 = vector.slice(optState[o],13,18)

end

function update()
	local t = Body.get_time()
	local t_diff = t - t_start
 	--if tbreak == 0 then
	if init == 0 then 
	  	if execution == 0 then
			print("======stepO: ",stepO,"==========")
			-- [[Middle Level Option]]--
			if stepO_pre ~= stepO then
			  	if math.mod(stepO,30) == 0 then
			  		print("findit",unpack(findit))
			  	end
			  	
				 	maxQ, maxid = util.max(Q[stateO])
				 	local ave = 0
				 	local extra = 0
				 	local multi = 1
				 	local first, last 
				 	if stateO < N_Binit then 
				 			first = N_Finit
				 			last = N_Fend
				 			multi = 1.8 
				 	else
				 			first = N_Binit
				 			last = N_Bend
				 			extra = N_Fend
				 	end
				 	
			 		if findit[stateO] == 0 then
				 		ave = (vector.sum(vector.slice(Q[stateO],first,last)) + Q[stateO][N_Bend+1] + Q[stateO][N_Bend+2] - Q[stateO][stateO])/(last - first + 2)
				 		sameVals = #(util.findidx(Q[stateO],maxQ))
				 		print("sameVals", sameVals)
		 				if (visitnum[stateO] == visitinit) and ((maxQ < ave + Qgap+Qgap*multi) or (sameVals > 1)) then
			 				notExp = util.findidx(util.concat(vector.slice(visits[stateO],first,last),vector.new({visits[stateO][N_Bend+1],visits[stateO][N_Bend+2]})),visitnum[stateO])
			 				print("Has Not Been Explored: ",unpack(notExp))
			 				if #notExp == 1 then
					 			visitnum[stateO] = visitnum[stateO] + 1
						 		if type(notExp) == "number" then
									o = notExp + extra
								else
									o = notExp[1] + extra
								end
			        		else 
			        			o = notExp[math.random(#notExp)] + extra
					 		end
					 		if (o > N_Fend) and (stateO < N_Binit) then
					 				o = o + (N_Bend - N_Binit + 1)
					 		end
					 	else -- when it has tried each action at least once | its maxQ is large enough
				 			print("AVERAGE:: ",ave )
					 		if (maxQ > (ave + Qgap*multi) and sameVals ==1) or  (vector.sum(visits[stateO]) > 3*(last-first+2)) then
				 				o = maxid
				 				print("FIND IT!!")
				 			  	if visits[stateO][o] > 50 then -- make sure it has tried at least three times
				 					findit[stateO] = o
				 				end
			 				else
						 		if stateO < N_Binit then 
						 			o = explore1[math.random(#explore1)]
						 		else
						 			o = explore2[math.random(#explore2)]
						 		end
						 	end
					 	end
					else
						o = findit[stateO]
					end

				 	stepO_pre = stepO
				end
				if (state_pre == stateO) and (o_pre == o) then
				  	print("STUCK, Interrupt!!")
				  	o = math.random(stateNum)
				end
				
				visits[stateO][o] = visits[stateO][o] + 1
				print("PROP:: ",prop, "  VISITS ", visits[stateO][o]-visitinit,"  STATE:::: ", stateO, " OPTION:::: ", o)
				alpha = prop*0.1/(1+visits[stateO][o])
				
				qLArm1 = vector.slice(optState[o],1,3)
			    qRArm1 = vector.slice(optState[o],4,6)
			    qLLeg1 = vector.slice(optState[o],7,12)
			    qRLeg1 = vector.slice(optState[o],13,18)
					execution = 1		 	

	    	else --- Action is Excuted! 
			  --print("execute?",t_diff/duration)
	      	if t_diff <= duration then 
	      		local ph = math.max(0,math.min(1, t_diff/duration))
					
		  		Body.set_larm_command((1-ph)*qLArm0 + ph * qLArm1)
				Body.set_rarm_command((1-ph)*qRArm0 + ph * qRArm1) 
				Body.set_lleg_command((1-ph)*qLLeg0 + ph * qLLeg1) 
				Body.set_rleg_command((1-ph)*qRLeg0 + ph * qRLeg1)  
				
				gyro = vector.new(Body.get_sensor_imuGyrRPY())*D2R
				angAcc = angAcc + vector.norm((gyro - gyro_pre))/(t - t_pre)
				gyro_pre = gyro
				t_pre = t

				--gyros = gyros + vector.norm(vector.new(Body.get_sensor_imuGyrRPY())*D2R)				
			elseif t_diff > 1.5*duration then --and hcm.get_state_proceed()==1 then
					--unix.usleep(1000000)
				qLArm0  = vector.new(Body.get_larm_position())  
				qRArm0  = vector.new(Body.get_rarm_position())  
				qLLeg0  = vector.new(Body.get_lleg_position())  
				qRLeg0  = vector.new(Body.get_rleg_position())

				q = qLArm0
				q = util.concat(q,qRArm0)
				q = util.concat(q,qLLeg0)
				q = util.concat(q,qRLeg0)

				quat = Body.get_sensor_quaternion()
				rpy  = Body.get_sensor_imuAngle()				  

				minH, supStateVec = Getup.getSup(quat, qLArm0, qRArm0, qLLeg0, qRLeg0, rpy[2]) -- ex) [1,0,0,1]^T
				minH_diff = abs(minH) - abs(minH_pre)
				rwO[stepO+1] = Getup.get_reward3(angAcc,rpy,q,minH_diff,rpy_pre,o,Rmax)
				if (o == state_pre) and (o_pre == stateO) then
					rwO[stepO+1] = rwO[stepO+1] - 0.001
				end
				Qpre = Q[stateO][o]

				----- STATE UPDATE -----------------------------------------------------------------------------------	
				rawStateVec = util.concat(qLArm0, qRArm0)
				rawStateVec = util.concat(rawStateVec, qLLeg0)
				rawStateVec = util.concat(rawStateVec, qRLeg0)
				rawStateVec = util.concat(rawStateVec,vector.new({rpy[1],rpy[2]}))

				state_pre = stateO
				o_pre = o
				stateO, prop= Getup.make_state(rawStateVec)

				Q[state_pre][o_pre] = (1-alpha) * Q[state_pre][o_pre] + alpha*(rwO[stepO+1] + (gamma)*util.max(Q[stateO]))
				--Q[stateO][o] = (1-alpha) * Q[stateO][o] + alpha*(rwO[stepO+1] + (gamma)*util.max(Q[stateO]))

				----- PRINTING -----------------------------------------------------------------------------------
				print("REWARD",rwO[stepO+1],"      ALPHA, ", alpha)
				print("Qupdated",Qpre, Q[state_pre][o_pre])  
				print("H ", abs(minH),"  ANG ACC ", angAcc)				  
				print("RPY: ", unpack(rpy))--rpy[1]/D2R,rpy[2]/D2R,rpy[3]/D2R)

				----- WRITING -----------------------------------------------------------------------------------	
				fid:write(stepO,"\t",	angAcc,"\t")
				for it=1,#rpy do
					fid:write(rpy[it],"\t")
				end
				for it=1,#q do
					fid:write(q[it],"\t")
				end
				fid:write(minH_diff,"\t",rpy_pre[2],"\t",rwO[stepO+1],"\t",state_pre,"\t",o,"\t")
				fid:write(Q[state_pre][o],"\n") 

				----- RESET -----------------------------------------------------------------------------------	
				minH_pre = minH	
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
				if (o == stateNum) and (abs(rpy[1])<0.05 and abs(rpy[2])<0.05) then
				  	if #(util.findidx(findit,0)) == 0 then
				  		print("DONE")
				  		return'done'
				  	else
					  	print("RESTART")
					  	--local yet = util.findidx(findit,0)
					  	o = math.random(stateNum)--yet[math.random(#yet)]
					  	init = 1
					  	qLArm1 = vector.slice(optState[o],1,3)
						  qRArm1 = vector.slice(optState[o],4,6)
						  qLLeg1 = vector.slice(optState[o],7,12)
						  qRLeg1 = vector.slice(optState[o],13,18)
					end
				end

	      	else 
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
			quat = Body.get_sensor_quaternion()
			rpy = {Body.get_sensor_imuAngle(1),Body.get_sensor_imuAngle(2),Body.get_sensor_imuAngle(3)}
			--print("quat", unpack(quat))
			print("RPY: ", rpy[1]/D2R,rpy[2]/D2R,rpy[3]/D2R)
			rawStateVec = util.concat(qLArm0, qRArm0)
			rawStateVec = util.concat(rawStateVec, qLLeg0)
			rawStateVec = util.concat(rawStateVec, qRLeg0)
			rawStateVec = util.concat(rawStateVec,vector.new({rpy[1],rpy[2]}))
			
			stateO, prop = Getup.make_state(rawStateVec)
			state_pre = stateNum
			o_pre = o
			minH, supStateVec = Getup.getSup(quat, qLArm0, qRArm0, qLLeg0, qRLeg0,rpy[2]) -- ex) [1,0,0,1]^T
			minH_pre = minH
			print("INITIAL HEIGHT ", abs(minH))
			--tbreak = 1
			--unix.usleep(1000000)
			t_start = t
			t_pre = t
			angAcc = 0
			gyro_pre = vector.new(Body.get_sensor_imuGyrRPY())*D2R
			rpy_pre = rpy
			init = 0 
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


	































