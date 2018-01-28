-- Author : Heejin Jeong ( University of Pennsylvania )
-- Start date : Feb 26th, 2016
-- Description: Learning stand-up motions using RL with symmetric features. In this file, the model includes symmetric states and actions, and the learning algorithm considers symmetric features. 
--				(Reference file: paper "Efficient Learning of Stand-up Motion for Humanoid Robots with Bilateral Symmetry", Heejin Jeong and Daniel D. Lee)

module(..., package.seeall);
name = ...;


require('Motion')
require('Body')
require('vector')
require('unix')
require('Kinematics')
require('Getup')
require('mcm')
require('Config')
require('util')
require('torch')
local Getup = require'Getup'


local sqrt 	= math.sqrt
local exp 	= math.exp
local pi	= math.pi
local abs	= math.abs
local D2R	= pi/180


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

local cwd = unix.getcwd();
local centroids = dofile(cwd.."/../Player/Motion/".."stateSym.lua");
local Qs = dofile(cwd.."/../Player/Motion/".."Qs.lua")
local stateNum = #centroids
local actNum 
---[ Variables ]---------------------------------------------------------------------------------
local qLArm0, qRArm0,qLLeg0,qRLeg0
local qLArm1, qRArm1,qLLeg1,qRLeg1
local o,osym
local prop = 1
--local visitinit = 0
--local visitnum = visitinit*vector.ones(stateNum)
local gyros ,gyros_pre ,angAcc 
local t,t_pre,t_start
local stepO, stepO_pre = 0
local pdfQ = {}
local sym = 0
-- local sym_pre = 0
local o_record
local hist_s = {}
local hist_a = {}
local hist_sym = {}

-- CONSTANTS
local init = 1
local init2 = 1
local execution
local duration = 1
local Rmax = 1
local alpha = 0
local gamma = 1
local GibbsTemp = 0.3

--local learned = {22,27,15,22,14,15,15,27,25,14,22,28,27}

function entry()
	fid  = io.open("test28.txt","w")
	fid2 = io.open("pdf28.txt","w")
  	Body.set_larm_hardness(0.7)
  	Body.set_rarm_hardness(0.7)
  	Body.set_lleg_hardness(0.7)
  	Body.set_rleg_hardness(0.7)
  	Body.set_head_hardness(0)
  
  -- TIME
	t_start = Body.get_time()
	t_pre = t_start
	gyros = 0
	gyros_pre = 0
	angAcc = 0
	
	execution = 0
  
	stepO = 2
	stepO_pre = 1

	-- Initialize Q, pdf, visits
	Q = Qs.Qsym
	actNum = #(Q[1])
	--visits = Qs.visits
	visits = util.constMatrix(stateNum,actNum,0)

	--Reward
	rwO = {}
  -- Get Data from the robot
	qLArm0 = vector.new(Body.get_larm_position())  
	qRArm0 = vector.new(Body.get_rarm_position())  
	qLLeg0 = vector.new(Body.get_lleg_position())  
	qRLeg0 = vector.new(Body.get_rleg_position()) 

	-- Initial Random Motion
	local choose = {11,2} 
	o = choose[math.random(2)]
	print("INITIAL: ", o)
  	qLArm1 = vector.slice(centroids[o],1,3)
  	qRArm1 = vector.slice(centroids[o],4,6)
  	qLLeg1 = vector.slice(centroids[o],7,12)
  	qRLeg1 = vector.slice(centroids[o],13,18)
end

function update()
	local t = Body.get_time()
	local t_diff = t - t_start
 	--if tbreak == 0 then
	if init == 0 then 
	  	if execution == 0 then
			print("======STEP: ",stepO,"==========")

			pdfQ = Gibbs(Q,GibbsTemp)
			--print(#pdfQ[stateO])
			--[[
			if stateO < 14 then -- FORWARD REGION
			  	while (o>13 and o<27) or (o>38) do
			  		o = sampling(pdfQ[stateO])
			  	end
			elseif stateO < 27 and stateO > 13 then -- BACKWARD REGION
				while (o < 14) or (o > 28 and o<39) do
			  		o = sampling(pdfQ[stateO])
			  	end
			else
				o = stateNum
			end
			]]
			print( "STATE:::: ", stateO, "SYM? ",sym)
			if stateO == 27 or stateO == 28 then
				o = 28
			--elseif stateO > 13 then 
			--	o = learned[stateO-13]
			else
				o = sampling(pdfQ[stateO])
				if sym == 1 then -- If s is a symmetric state, a selected action should be symmetrized 
					o = symmetric(o)
				end
			end		
			--hist_a[stepO] = o			
			
			print(" ACTION:: ", o,"with p=",pdfQ[stateO][o])

			if o <(stateNum+1) then 
				qLArm1 = vector.slice(centroids[o],1,3)
			    qRArm1 = vector.slice(centroids[o],4,6)
			    qLLeg1 = vector.slice(centroids[o],7,12)
			    qRLeg1 = vector.slice(centroids[o],13,18)
			    
			else --- When o is Symmetric Action
				-- osym := symmetric action number
				osym = symmetric(o)
				qRArm1 = vector.slice(centroids[osym],1,3)
			    qLArm1 = vector.slice(centroids[osym],4,6)
			    qRLeg1 = vector.slice(centroids[osym],7,12)
			    qLLeg1 = vector.slice(centroids[osym],13,18)

			    qLArm1[2] = -qLArm1[2]
			    qRArm1[2] = -qRArm1[2]
			    qLLeg1[1] = -qLLeg1[1]
			    qRLeg1[1] = -qRLeg1[1]
			    qLLeg1[2] = -qLLeg1[2]
			    qRLeg1[2] = -qRLeg1[2]
			    qLLeg1[6] = -qLLeg1[6]
			    qRLeg1[6] = -qRLeg1[6]
			end
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

			elseif t_diff > 2*duration then 
					
				qLArm0  = vector.new(Body.get_larm_position())  
			  	qRArm0  = vector.new(Body.get_rarm_position())  
		  		qLLeg0  = vector.new(Body.get_lleg_position())  
			  	qRLeg0  = vector.new(Body.get_rleg_position())

			  	---- GET REWARD 
				q = vector.concat(qLArm0,qRArm0)
				q = vector.concat(q,qLLeg0)
				q = vector.concat(q,qRLeg0)

				quat = Body.get_sensor_quaternion()
				rpy  = Body.get_sensor_imuAngle()				  
					  
			  	minH, supStateVec = Getup.getSup(quat, qLArm0, qRArm0, qLLeg0, qRLeg0, rpy[2]) -- ex) [1,0,0,1]^T
			  	minH_diff = abs(minH) - abs(minH_pre)
			  	rwO[stepO] = Getup.get_reward5(angAcc,rpy,q,minH_diff,rpy_pre,o,Rmax)
			  	
			  	-- SYMMETRIC, TO RECORD
			  	if sym == 1 then
			  		o_record = symmetric(o)
			  	else
			  		o_record = o
			  	end
			  	

				-- LOCAL OPTIMA
				--if hist_sym[stepO-1] == sym and hist_s[stepO-1] == o then
				if hist_sym[stepO-1] == sym and hist_s[stepO-1] == o_record then
					local maxV, maxid = util.max(pdfQ[o_record])
					if maxid == hist_a[stepO-1] then
						rwO[stepO] = rwO[stepO] - 3--*0.1*(1+visits[stateO][o_record])
						print("LOCAL OPTIMAAAAAAAAAAAAAAAAA")
					end
				elseif hist_sym[stepO-1] ~= sym then
					if hist_s[stepO-1] == symmetric(o_record) then 
						local maxV, maxid = util.max(pdfQ[hist_s[stepO-1]])
						if maxid == hist_a[stepO-1] then
							rwO[stepO] = rwO[stepO] - 3--*0.1*(1+visits[stateO][o_record])
							print("TRAPPP")
						end
					end
				elseif (stepO > 2) and (hist_sym[stepO-2] == sym) and (hist_s[stepO-2] == stateO) and (hist_a[stepO-2] == o_record) then
					rwO[stepO] = rwO[stepO] - 3--*0.1*(1+visits[stateO][o_record])
				end
				----
				--visits[stateO][o] = visits[stateO][o] + 1
				visits[stateO][o_record] = visits[stateO][o_record] + 1
				alpha = 0.5*prop/(1+visits[stateO][o_record])
				print("VISITS ", visits[stateO][o_record],"ALPHA, ", alpha, "	PROP, ",prop)

				---- History
				hist_s[stepO] = stateO
				hist_a[stepO] = o_record
				hist_sym[stepO] = sym
				
			    ----- STATE UPDATE -----------------------------------------------------------------------------------	
				rawStateVec = vector.concat(q,vector.new({rpy[1],rpy[2]}))
				stateO, prop, sym= Getup.make_stateSYM(rawStateVec)

				if (hist_s[stepO] == stateO) and (stateO ~= 28) then
					rwO[stepO] = rwO[stepO] - 2--*0.1*(1+visits[hist_s[stepO]][hist_a[stepO]])
				end


				-- Q UPDATE
				Qpre = Q[hist_s[stepO]][hist_a[stepO]]
				Q[hist_s[stepO]][hist_a[stepO]] = (1-alpha) * Q[hist_s[stepO]][hist_a[stepO]] + alpha*(rwO[stepO] + (gamma)*util.max2(Q[stateO]))

				----- PRINTING -----------------------------------------------------------------------------------
				print("REWARD",rwO[stepO])
				print("Qupdated ",Qpre, "Q(",hist_s[stepO],",",hist_a[stepO],"): ",Q[hist_s[stepO]][hist_a[stepO]]) 
				print("Qnext max: ", util.max2(Q[stateO])) 
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
			  	fid:write(minH_diff,"\t",rpy_pre[2],"\t",rwO[stepO],"\t",hist_s[stepO],"\t",hist_a[stepO],"\t")
			  	fid:write(Q[hist_s[stepO]][hist_a[stepO]],"\t",alpha,"\t",visits[hist_s[stepO]][hist_a[stepO]],"\t",util.max2(Q[stateO]),"\t",hist_sym[stepO],"\n") 

			  	fid2:write(hist_s[stepO],"\t")
			  	for itt=1,#(pdfQ[hist_s[stepO]]) do
			  		fid2:write(pdfQ[hist_s[stepO]][itt],"\t")
			  	end
			  	fid2:write("\n")

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
			  	if (o == stateNum) and (abs(rpy[1])<0.4 and abs(rpy[2])<0.5) then
			  		--[[
			  		if util.max2(pdfQ[state_pre]) > 0.8 then
			  			print("DONE")
			  			return'done'
			  		else]]
				  		print("RESTART")
				  		--local yet = util.findidx(findit,0)
				  		local choose = {11,2} 
				  		o = choose[math.random(2)]--math.random(26)--yet[math.random(#yet)]
				  		init = 1
				  		init2 = 1
				  		qLArm1 = vector.slice(centroids[o],1,3)
					  	qRArm1 = vector.slice(centroids[o],4,6)
					  	qLLeg1 = vector.slice(centroids[o],7,12)
					  	qRLeg1 = vector.slice(centroids[o],13,18)
					--end
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
		
		elseif t_diff > 2*duration then --and hcm.get_state_proceed()==1 then
			qLArm0  = vector.new(Body.get_larm_position())  
		    qRArm0  = vector.new(Body.get_rarm_position())  
	  		qLLeg0  = vector.new(Body.get_lleg_position())  
		  	qRLeg0  = vector.new(Body.get_rleg_position())
		  	if init2 == 1 then
		  		o = math.random(10)--yet[math.random(#yet)]
		  		init = 1
		  		qLArm1 = vector.slice(centroids[o],1,3)
			  	qRArm1 = vector.slice(centroids[o],4,6)
			  	qLLeg1 = vector.slice(centroids[o],7,12)
			  	qRLeg1 = vector.slice(centroids[o],13,18)
			  	init2 = 0
			  	t_start = t
				t_pre = t
			else 
			  	-- State Update		  	
			  	quat = Body.get_sensor_quaternion()
			  	rpy = {Body.get_sensor_imuAngle(1),Body.get_sensor_imuAngle(2),Body.get_sensor_imuAngle(3)}

				rawStateVec = vector.concat(qLArm0, qRArm0)
				rawStateVec = vector.concat(rawStateVec, qLLeg0)
				rawStateVec = vector.concat(rawStateVec, qRLeg0)
				rawStateVec = vector.concat(rawStateVec,vector.new({rpy[1],rpy[2]}))
				
				stateO, prop, sym = Getup.make_stateSYM(rawStateVec)
				hist_s[stepO_pre] = stateNum
				hist_a[stepO_pre] = o
				minH, supStateVec = Getup.getSup(quat, qLArm0, qRArm0, qLLeg0, qRLeg0,rpy[2]) -- ex) [1,0,0,1]^T
				minH_pre = minH
				print("INITIAL HEIGHT ", abs(minH))
				print("RPY: ", rpy[1]/D2R,rpy[2]/D2R,rpy[3]/D2R)
				
				-- RESET
				t_start = t
				t_pre = t
				angAcc = 0
				gyro_pre = vector.new(Body.get_sensor_imuGyrRPY())*D2R
				rpy_pre = rpy
				init = 0 
				--stepO_pre = stepO
				--stepO = stepO +1
			end
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


function sampling(p)
	local cdf={}
	cdf[1]=p[1]
	for i=2,#p do
		cdf[i]=cdf[i-1]+p[i]
	end
	torch.manualSeed(os.time())
	local y = torch.uniform()
	--print("sample p",y)
	local j = 0
	local tmp = -1
	while j < (#p) and tmp < 0 do
		j = j+1
		tmp = cdf[j]-y
	end	
	--print(unpack(p))
	return j
end

function rowsum(M)
	local s={}
	for i=1,#M do
		s[i] = 0
		for j=1,#(M[i]) do
			s[i] = s[i]+M[i][j]
		end
	end
	return s
end

function Gibbs(M,temp)
	--local s = {}
	local P = {}
	for i=1,#M do
		P[i] = {}
		local s = 0
		local v = {}
		for j=1,#(M[i]) do
			v[j] = exp(M[i][j]/temp)
			s = s + v[j]
		end
		P[i] = vector.divnum(v,s)
	end
	return P
end

function symmetric(a)
	local asym 
	if a < 11 then 
		asym = a+28
	elseif a > 13 and a<22 then
		asym = a+25
	elseif a > 28 and a<39 then
		asym = a-28 
	elseif a>38 then
		asym = a-25
	else
		asym = a
	end
	return asym
end
























