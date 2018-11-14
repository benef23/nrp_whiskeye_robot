
import numpy as np
import math

from model_pars import *



###################################### HELPER FCNS #######################################

def rotate( point_in, axis, angle ):

	point_out = np.array([0.0, 0.0, 0.0])

	c = np.cos( angle )
	s = np.sin( angle )

	if 'x' in axis:
		point_out[1] = c*point_in[1] - s*point_in[2]
		point_out[2] = s*point_in[1] + c*point_in[2]
		point_out[0] = point_in[0]

	elif 'y' in axis:
		point_out[2] = c*point_in[2] - s*point_in[0]
		point_out[0] = s*point_in[2] + c*point_in[0]
		point_out[1] = point_in[1]

	elif 'z' in axis:
		point_out[0] = c*point_in[0] - s*point_in[1]
		point_out[1] = s*point_in[0] + c*point_in[1]
		point_out[2] = point_in[2]

	else:
		raise ValueError( 'invalid axis' )

	return point_out

def rotate_fwd( lhs, link ):
	return rotate( lhs, link.axis, link.angle )

def rotate_rev( lhs, link ):
	return rotate( lhs, link.axis, -link.angle )



################################################################
##	KC
################################################################

KC_ANGLE_UNCONSTRAINED = 4.0 * np.pi

class KinematicLink:

	def __init__( self, name, trans, axis, angle_ini, pars ):

		self.name = name
		self.translation = trans
		self.axis = axis
		self.angle = angle_ini

		if pars is None:
			self.angle_min = angle_ini
			self.angle_max = angle_ini
			self.angle_fix = angle_ini
		else:
			self.angle_min = pars[0]
			self.angle_max = pars[1]
			self.angle_fix = pars[2]

	def toParentAbs(self, pos):
		pos = rotate_fwd(pos, self)
		return pos + self.translation

	def toParentRel(self, direction):
		return rotate_fwd(direction, self)

	def fromParentAbs(self, pos):
		pos = pos - self.translation
		return rotate_rev(pos, self)

	def fromParentRel(self, direction):
		return rotate_rev(direction, self)

	def push(self, pushpos_i, pushvec):

		# pushpos: the "push point", the point we want to move
		# pushvec: the "push vector", the amount we want it to move by
		#
		# "i": initial (before the push is applied)
		# "f": final (after the push is applied)
		#
		# the implementation aims to move the push point onto the
		# "pushed point", defined as:
		#
		# pushpos_f = pushpos_i + pushvec
		#
		# in the FOR defined by this link.
		#
		# owing to constraints (joint axis, angle limits) we can't generally
		# find a link movement that achieves this. instead, we do the best we
		# can, and can define the "actual pushed point" as:
		#
		#	pushpos_f_act = pushpos_i + pushvec_act
		#
		# having done this, we can pass up the "remaining push vector", defined
		# as:
		#
		#	pushvec_rem = pushvec - pushvec_act
		#
		# into the parent frame for further processing.

		#	compute proposed pushed point
		pushpos_f = pushpos_i + pushvec

		#	amount we will change joint angle by
		angle_delta = 0.0

		# if not fixed
		if self.angle_fix == KC_ANGLE_UNCONSTRAINED:

			# calculate movement of link's joint required to bring
			# pushpos_i as close to pushpos_f as possible, given
			# only this joint's axis of rotation
			if self.axis == 'x':
				angle_delta = math.atan2(pushpos_f[2], pushpos_f[1]) - math.atan2(pushpos_i[2], pushpos_i[1])
			if self.axis == 'y':
				angle_delta = math.atan2(pushpos_f[0], pushpos_f[2]) - math.atan2(pushpos_i[0], pushpos_i[2])
			if self.axis == 'z':
				angle_delta = math.atan2(pushpos_f[1], pushpos_f[0]) - math.atan2(pushpos_i[1], pushpos_i[0])

		# if fixed
		else:

			# approach angle_fix
			angle_delta = (self.angle_fix - self.angle) * 0.1

		# minimise absolute value, solely for computational stability
		if angle_delta > math.pi:
			angle_delta -= 2 * math.pi
		if angle_delta < -math.pi:
			angle_delta += 2 * math.pi

		# store old joint angle
		angle_i = self.angle

		# propose new joint angle based on this result
		angle_f = self.angle + angle_delta

		# if not fixed
		if self.angle_fix == KC_ANGLE_UNCONSTRAINED:

			# constrain that proposed joint angle by joint limits
			if not self.angle_min == KC_ANGLE_UNCONSTRAINED:
				angle_f = max(angle_f, self.angle_min)
			if not self.angle_max == KC_ANGLE_UNCONSTRAINED:
				angle_f = min(angle_f, self.angle_max)

			# recalculate angle_delta after these constraints have been applied
			angle_delta = angle_f - angle_i

		# perform the rotation to see how much pushpos has moved
		pushpos_f_act = rotate(pushpos_i, self.axis, angle_delta)

		# and compute actual pushvec
		pushvec_act = pushpos_f_act - pushpos_i

		# and remaining pushvec
		#
		# NB: this is _still_ in the old FOR, before the link
		# joint is adjusted (i.e. with "angle_i" in the joint).
		pushvec_rem = pushvec - pushvec_act

		# transform remaining pushvec into parent frame, for chaining,
		# through the old state of the link
		self.angle = angle_i
		pushvec_rem_parent = self.toParentRel(pushvec_rem)

		# lay in actual new angle to implement the push operation
		self.angle = angle_f

		# transform push point into parent frame too. note that
		# it's in the same place as it was - the link has moved,
		# but the push point (e.g. fovea) is still at the same
		# location within it.
		pushpos_i_parent = self.toParentAbs(pushpos_i)

		# return these for propagation
		return (pushpos_i_parent, pushvec_rem_parent)

class KinematicChain:

	def __init__( self, link_desc_array ):

		# data
		self.link = []
		self.linkNames = []

		# create links
		for link_desc in link_desc_array:

			# extract init data
			name = link_desc[0]
			trans = link_desc[1]
			axis = link_desc[2]
			angle_ini = link_desc[3]
			pars = link_desc[4]

			# add link itself
			self.link.append(KinematicLink(name, trans, axis, angle_ini, pars))
			self.linkNames.append(name)

	def zeroPose(self):
		self.link[0].translation = np.array([0.0, 0.0, 0.0])
		self.link[0].angle = 0.0

	def getPose(self):
		return (self.link[0].translation[0:2], self.link[0].angle)

	def setConfig( self, config ):
		for i in range(1, len(self.link)):
			self.link[i].angle = config[i-1]

	def getConfig(self):
		config = []
		for i in range(1, len(self.link)):
			config.append(self.link[i].angle)
		return config

	def getState( self ):
		return [self.getPose(), self.getConfig()]

	def indexFrame(self, text):
		if text == 'PARENT':
			return -1
		else:
			return self.linkNames.index(text)

	def changeFrameAbs( self, inFrame_text, outFrame_text, pos ):

		inFrame = self.indexFrame(inFrame_text)
		outFrame = self.indexFrame(outFrame_text)

		# ascend chain
		if outFrame > inFrame:
			for i in range(inFrame, outFrame):
				link = self.link[i+1]
				pos = link.fromParentAbs(pos)

		# descend chain
		elif inFrame > outFrame:
			for i in range(inFrame, outFrame, -1):
				link = self.link[i]
				pos = link.toParentAbs(pos)

		return pos

	def changeFrameRel( self, inFrame_text, outFrame_text, pos ):

		inFrame = self.indexFrame(inFrame_text)
		outFrame = self.indexFrame(outFrame_text)

		# ascend chain
		if outFrame > inFrame:
			for i in range(inFrame, outFrame):
				link = self.link[i+1]
				pos = link.fromParentRel(pos)

		# descend chain
		elif inFrame > outFrame:
			for i in range(inFrame, outFrame,-1):
				link = self.link[i]
				pos = link.toParentRel(pos)

		return pos

	def camera2head( camera, pos ):

		# Apply divergence
		# -ve once, because azimuth of camera not object
		# -ve again, because we're inverting camera mapping
		# +ve overall
		if camera == 'r':
			azim = WHISKY_CAM_DIVERGENCE
		elif camera == 'l':
			azim = -WHISKY_CAM_DIVERGENCE
		else:
			raise ValueError( 'camera not r or l ' )

		pos = rotate( pos, 'z', azim )

		# Apply elevation
		# As above, two -ves mmake +ve.
		# Camera elevation defined as +ve == upwards
		# Counter to +ve == rotation from z to x RH convention.
		# So end up with a -ve
		elev = WHISKY_CAM_ELEVATION
		pos = rotate( pos, 'y', -elev )

		# Apply translation ( for left camera, y value gets inverted )
		pos[0] += WHISKY_LOC_EYE_X
		if camera == 'r':
			pos[1] += WHISKY_LOC_EYE_Y
		else:
			pos[1] -= WHISKY_LOC_EYE_Y
		pos[2] += WHISKY_LOC_EYE_Z

		return pos

	def push(self, push):

		# pushes in frame "-1" are null
		if push.frame == -1:
			return

		# if push is a velocity, reduce it to a position change
		push = push.resolve()

#		This function applies the push vector "pushvec" to the pushpoint
#		"pushpos" in the link frame "pushlink". This results in a change
#		in the state of the KC which will include changes
#		to the rotation angle of every link in the chain as well as a
#		change to the translation vector of the final link, which is
#		assumed to be the unconstrained link (usually, BODY is mobile
#		in the x/y plane of WORLD).
#
#		The changes to the constrained links are expected to be
#		implemented using position servos, so the values for their
#		configuration stored in the chain are sufficient for the caller
#		to implement the changes to the model in hardware. For the
#		unconstrained link, however, the hardware uses velocity servos
#		(i.e. the wheel speed controllers). The caller must observe the
#		changes to kc across any pushes that are applied to recover these.
#
#		Thus, use this function as follows:
#
#		kc_m.zeroPose()
#		...apply one or more pushes...
#		state = kc_m.getState()
#
#		The result in "state" includes the change to the BODY FOR across
#		the pushes and the new configuration of the remaining joints.
#
#		NB: the terms "frame" and "link" are used pretty much
#		interchangeably throughout this document. The "frame" is the
#		frame-of-reference (FOR), which is associated with a physical
#		link (an actual rigid member, jointed to others such).

		# test push
		#push.vec = np.array([0.0, 0.01, 0.0])

		# walk the push down the chain to the zeroth link
		for i in range(push.frame, 0, -1):
			(push.pos, push.vec) = self.link[i].push(push.pos, push.vec)

#		After walking the servo joints of the KC, we will - in general
#		- be left with a non-zero push-vector in BODY. Servicing this
#		requires a movement of BODY in WORLD (provided BODY is mobile).
#
#		The treatment is different for platforms with and without
#		non-holonomic constraints. Holonomic (omnidirectional)
#		platforms can minimise their total movement by treating
#		the final left-over vector as:
#
#			a) rotate until pushvec points away from or towards
#				platform centre point.
#			b) move platform centre point to zero pushvec.
#
#		Noting that (a) is the same operation as is used in pushing
#		the servo joints, we can service it with the same code.
#
#		Non-holonomic platforms cannot do this, because (b) may
#		not be in a direction they can move. Instead, they do:
#
#			a) rotate until pushvec is aligned with drive direction.
#			b) move platform centre point to zero pushvec.
#
#		Finally, the above treatment for NH platforms is not stable
#		if the dr component is not respected. So, for gaze changes only
#		(dr constrained to be zero), we use a different approach.

		# holonomic
		if True:

			# store initial orientation of mobile frame
			body_ori_i = self.link[0].angle

			# push BODY in WORLD
			(push.pos, push.vec) = self.link[0].push(push.pos, push.vec)

			# store final orientation of mobile frame
			body_ori_f = self.link[0].angle

			# finally, the remaining translation is now in WORLD, after the
			# above call to push(). we just need to pop it back up to BODY to
			# give the caller the result they need.
			poseChange = self.link[0].fromParentRel(push.vec)

			# measure how much BODY was rotated by the push
			# element 2 of poseChange is theta (x, y, theta)
			dtheta = body_ori_f - body_ori_i

			# but we implement this by driving link[0]
			poseChange[2] = 0.0 # z movement not allowed!
			self.link[0].translation += poseChange
			# ...theta already effected, above

		# non-holonomic
		else:

			# TODO: port the C code below for this case
			pass

		#print np.round(np.degrees([self.link[0].angle, self.link[1].angle, self.link[2].angle, self.link[3].angle]))

"""


	#else

		if (push->flags & MIRO_BODY_PUSH_ZERO_POSE_DR)
		{
			/*
				Special treatment for gaze only. We can't run the full algo
				because it's unstable unless we do the dr component that
				comes out of it. Instead, since we are only correcting gaze
				direction in azimuth in this case, we use a different tack.
				Having completed the push through the system, we transform
				the remaining pushvec back up into HEAD. There, we measure
				the azimuthal error remaining. We implement that in BODY,
				which will approximately correct it in HEAD, also.

				NB: We could compute it exactly, it's not very onerous, but
				I think this computation is a pretty good approx providing
				the gaze range is large, and it's also I think stable, giving
				a slightly smaller dtheta than the correct computation.
			*/

			//	transform pushpos and pushvec back up to HEAD
			kc_changeFrameAbs(kc, MIRO_LINK_FOOT, MIRO_LINK_HEAD, &pushpos);
			kc_changeFrameRel(kc, MIRO_LINK_FOOT, MIRO_LINK_HEAD, &pushvec);

			//	P is the pushpos and V is the pushvec
			float3 P = pushpos;
			float3 V = pushvec;

			//	compute pushed point P' = P + V
			float3 P_prime = P;
			float3_add_unary(&P_prime, &V);

			//	fill debug
			if (debug)
			{
				debug->P = P;
				debug->V = V;
				debug->P_prime = P_prime;
			}

			//	measure azimuth of P and P_prime
			float a = atan2f_accurate(P[1], P[0]);
			float b = atan2f_accurate(P_prime[1], P_prime[0]);

			//	difference is change in azimuth to bring them in line
			poseChange.dr = 0.0f;
			poseChange.dtheta = b - a;
		}

		else
		{
			/*
				Non-holonomic treatment; see Fig NH in kc.png, alongside.

				To solve for a non-holonomic (diff. drive) platform, we can only
				move the platform exactly forwards or backwards, after making a
				rotation of our choice. Therefore, our goal is to rotate the
				platform so that the pushpoint P is directly "behind" (in the
				robot's eyes) the target point P' = P + V, with V the pushvec.
				Having done this, we can then push the platform forward by an
				amount "y" until P arrives at P'. We label the image of P' after
				the rotation of the robot Q. We know that Q has the same
				x coordinate as P, because that is its definition (directly
				behind or in front of P means P[0] == Q[0]). We also know it has
				the same distance from the origin as P', because it's a rotation
				of P' around the origin (the axis centre, for BODY). From these
				two bits of information, we can determine its y coordinate; more
				saliently, we can determine its distance ahead of P, which will
				be our "dr" for BODY. Finally, we can recover the rotation
				needed as equal to "c", with c = a - b (aka dtheta for BODY).
			*/

			//	P is the pushpos and V is the pushvec
			float3 P = pushpos;
			float3 V = pushvec;

			//	compute pushed point P' = P + V
			float3 P_prime = P;
			float3_add_unary(&P_prime, &V);

			//	fill debug
			if (debug)
			{
				debug->P = P;
				debug->V = V;
				debug->P_prime = P_prime;
			}

			//	measure P' radius, denoted r in Fig NH
			float r = sqrtf(P_prime[0] * P_prime[0] + P_prime[1] * P_prime[1]);

			//	compute pushed point after rotation by the desired amount
			//	which we have denoted Q
			float y = P[1];
			float x_sq = r*r - y*y;

			//	NB: There are pushes we will not be able to satisfy. See Fig
			//	NH-FAIL, where r is smaller than y. In this case, x_sq will
			//	come out as -ve. I intend to take measures in the MPG to make
			//	sure this doesn't happen in practice; if it makes it this far,
			//	rather than take the sqrt(-ve) and throw a NaN in the works,
			//	we'll just abandon the computation and raise a warning.
			if (x_sq < 0)
			{
				__WARNING(XSQ_NEGATIVE);
				poseChange.dr = 0.0f;
				poseChange.dtheta = 0.0f;
			}

			else
			{
				//	compute rotation required to achieve that
				float x = sqrtf(x_sq);
				float b = atan2f_accurate(P_prime[1], P_prime[0]);
				float a = atan2f_accurate(y, x);
				float c = b - a;

				//	derive pose change
				poseChange.dr = x - P[0];
				poseChange.dtheta = c;
			}
		}

	#endif // HOLONOMIC

		//	add up pose change from any action that pushes the KC
		__BODY.accum_poseChange.dr += poseChange.dr;
		__BODY.accum_poseChange.dtheta += poseChange.dtheta;

"""



################################################################
##	CONSTANTS
################################################################

def kc_whiskeye():
	return KinematicChain([
			['BODY',
				np.array([0.0, 0.0, 0.0]),
				'z',
				0.0,
				[KC_ANGLE_UNCONSTRAINED, KC_ANGLE_UNCONSTRAINED, KC_ANGLE_UNCONSTRAINED] ],
			['NECK',
				np.array([WHISKY_LOC_LIFT_X, WHISKY_LOC_LIFT_Y, WHISKY_LOC_LIFT_Z]),
				'y',
				WHISKY_LIFT_INI_RAD,
				[WHISKY_LIFT_MIN_RAD, WHISKY_LIFT_MAX_RAD, KC_ANGLE_UNCONSTRAINED] ],
			['GMBL',
				np.array([WHISKY_LOC_PITCH_X, WHISKY_LOC_PITCH_Y, WHISKY_LOC_PITCH_Z]),
				'y',
				WHISKY_PITCH_INI_RAD,
				[WHISKY_PITCH_MIN_RAD, WHISKY_PITCH_MAX_RAD, KC_ANGLE_UNCONSTRAINED] ],
			['HEAD',
				np.array([WHISKY_LOC_YAW_X, WHISKY_LOC_YAW_Y, WHISKY_LOC_YAW_Z]),
				'z',
				WHISKY_YAW_INI_RAD,
				[WHISKY_YAW_MIN_RAD, WHISKY_YAW_MAX_RAD, KC_ANGLE_UNCONSTRAINED] ]
		])

def kc_whisker(row, col, row_board, whisker):
	rx = row_board[0]
	ry = row_board[1]
	rz = row_board[2]
	xyz = np.array(row_board[3:6])

	rw = whisker[0]
	whisker_x = whisker[1]
	whisker_y = whisker[2]
	whisker_z = whisker[3]

	initial_theta = 0.0

	return KinematicChain([
		['ROWT',
			xyz,
			'x', 0, None ],
		['ROTX',
			np.array([0, 0, 0]),
			'x', rx, None ],
		['ROTY',
			np.array([0, 0, 0]),
			'y', ry, None ],
		['ROW_BOARD',
			np.array([0, 0, 0]),
			'z', rz, None ],
		['WHISKER_BASE',
			np.array([whisker_x, whisker_y, whisker_z]),
			'x', rw, None ],
		['WHISKER',
			np.array([0.0, 0.0, 0.0]),
			'y',
			initial_theta,
			[KC_ANGLE_UNCONSTRAINED, KC_ANGLE_UNCONSTRAINED, KC_ANGLE_UNCONSTRAINED] ]
		])

def kc_whiskers(pars):
	kc = []
	for row in pars.rows:
		w = []
		for col in pars.cols:
			w.append(kc_whisker(row, col, pars.row_boards[row], pars.whiskers[col]))
		kc.append(w)
		#### validation ####
		#xyz = np.array([0.0, 0.0, 0.0])
		#xyz = w[0].changeFrameAbs('ROW_BOARD', 'PARENT', xyz)
		#print xyz
		#### validation ####
	return kc



