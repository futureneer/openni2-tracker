/*********************************************************************
* Software License Agreement (BSD License)
*
* Copyright (c) 2013, Johns Hopkins University
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* * Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above
* copyright notice, this list of conditions and the following
* disclaimer in the documentation and/or other materials provided
* with the distribution.
* * Neither the name of the Johns Hopkins University nor the names of its
* contributors may be used to endorse or promote products derived
* from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/*
 * Author: Kelleher Guerin, futureneer@gmail.com, Johns Hopkins University
 */

// ROS Dependencies
#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_broadcaster.h>
#include <kdl/frames.hpp>
// NITE Dependencies
#include "NiTE.h"
#include "NiteSampleUtilities.h"
// Tracker Messages
#include <openni2_tracker/TrackerUser.h>
#include <openni2_tracker/TrackerUserArray.h>

#define MAX_USERS 10
bool g_visibleUsers[MAX_USERS] = {false};
nite::SkeletonState g_skeletonStates[MAX_USERS] = {nite::SKELETON_NONE};

#define USER_MESSAGE(msg) \
	{printf("[%08llu] User #%d:\t%s\n",ts, user.getId(),msg);}

void updateUserState(const nite::UserData& user, unsigned long long ts)
{
	if (user.isNew())
		USER_MESSAGE("New")
	else if (user.isVisible() && !g_visibleUsers[user.getId()])
		USER_MESSAGE("Visible")
	else if (!user.isVisible() && g_visibleUsers[user.getId()])
		USER_MESSAGE("Out of Scene")
	else if (user.isLost())
		USER_MESSAGE("Lost")

	g_visibleUsers[user.getId()] = user.isVisible();


	if(g_skeletonStates[user.getId()] != user.getSkeleton().getState())
	{
		switch(g_skeletonStates[user.getId()] = user.getSkeleton().getState())
		{
		case nite::SKELETON_NONE:
			USER_MESSAGE("Stopped tracking.")
			break;
		case nite::SKELETON_CALIBRATING:
			USER_MESSAGE("Calibrating...")
			break;
		case nite::SKELETON_TRACKED:
			USER_MESSAGE("Tracking!")
			break;
		case nite::SKELETON_CALIBRATION_ERROR_NOT_IN_POSE:
		case nite::SKELETON_CALIBRATION_ERROR_HANDS:
		case nite::SKELETON_CALIBRATION_ERROR_LEGS:
		case nite::SKELETON_CALIBRATION_ERROR_HEAD:
		case nite::SKELETON_CALIBRATION_ERROR_TORSO:
			USER_MESSAGE("Calibration Failed... :-|")
			break;
		}
	}
}

int main(int argc, char** argv)
{

	ros::init(argc, argv, "openni2_tracker");
	ros::NodeHandle nh;
	ros::NodeHandle pnh("~"); //private node handler
	std::string tf_prefix, relative_frame = "";

	// Get Tracker Parameters
	if(!pnh.getParam("tf_prefix", tf_prefix)){
		ROS_ERROR("tf_prefix not found on Param Server! Check your launch file!");
		return -1;
	}
	if(!pnh.getParam("relative_frame", relative_frame)){
		ROS_ERROR("relative_frame not found on Param Server! Check your launch file!");
		return -1;
	}

	// NITE Stuff
	nite::UserTracker userTracker;
	nite::Status niteRc;
	nite::NiTE::initialize();
	niteRc = userTracker.create();
	if (niteRc != nite::STATUS_OK){
		printf("Couldn't create user tracker\n");
		return 3;
	}

	// Loop
	printf("\nStart moving around to get detected...\n(PSI pose may be required for skeleton calibration, depending on the configuration)\n");

	nite::UserTrackerFrameRef userTrackerFrame;
	// while (!wasKeyboardHit()){
	ros::Rate rate(100.0);
	while (nh.ok()){
		niteRc = userTracker.readFrame(&userTrackerFrame);
		if (niteRc != nite::STATUS_OK){
			printf("Get next frame failed\n");
			continue;
		}

		const nite::Array<nite::UserData>& users = userTrackerFrame.getUsers();
		for (int i = 0; i < users.getSize(); ++i){
			const nite::UserData& user = users[i];
			updateUserState(user,userTrackerFrame.getTimestamp());
			if (user.isNew()){
				userTracker.startSkeletonTracking(user.getId());
			} else if (user.getSkeleton().getState() == nite::SKELETON_TRACKED){
				std::map<std::string, nite::SkeletonJoint> named_joints;
				
				named_joints["hea"] = (user.getSkeleton().getJoint(nite::JOINT_HEAD) );
				named_joints["nek"] = (user.getSkeleton().getJoint(nite::JOINT_NECK) );
				named_joints["lsh"] = (user.getSkeleton().getJoint(nite::JOINT_LEFT_SHOULDER) );
				named_joints["rsh"] = (user.getSkeleton().getJoint(nite::JOINT_RIGHT_SHOULDER) );
				named_joints["lel"] = (user.getSkeleton().getJoint(nite::JOINT_LEFT_ELBOW) );
				named_joints["rel"] = (user.getSkeleton().getJoint(nite::JOINT_RIGHT_ELBOW) );
				named_joints["lhn"] = (user.getSkeleton().getJoint(nite::JOINT_LEFT_HAND) );
				named_joints["rhn"] = (user.getSkeleton().getJoint(nite::JOINT_RIGHT_HAND) );
				named_joints["tor"] = (user.getSkeleton().getJoint(nite::JOINT_TORSO) );
				named_joints["lhp"] = (user.getSkeleton().getJoint(nite::JOINT_LEFT_HIP) );
				named_joints["rhp"] = (user.getSkeleton().getJoint(nite::JOINT_RIGHT_HIP) );
				named_joints["lkn"] = (user.getSkeleton().getJoint(nite::JOINT_LEFT_KNEE) );
				named_joints["rkn"] = (user.getSkeleton().getJoint(nite::JOINT_RIGHT_KNEE) );
				named_joints["lft"] = (user.getSkeleton().getJoint(nite::JOINT_LEFT_FOOT) );
				named_joints["rft"] = (user.getSkeleton().getJoint(nite::JOINT_RIGHT_FOOT) );

				// if (head.getPositionConfidence() > .5)
				// printf("%d. (%5.2f, %5.2f, %5.2f)\n", user.getId(), head.getPosition().x, head.getPosition().y, head.getPosition().z);
			}
		}
		rate.sleep();
	}
	nite::NiTE::shutdown();
}
