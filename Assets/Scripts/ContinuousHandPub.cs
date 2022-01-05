using System;
using System.Collections;
using System.Linq;
using RosMessageTypes.Geometry;
using RosMessageTypes.Std;
using RosMessageTypes.InterbotixMoveitInterface;
using RosMessageTypes.InterbotixHandJoy;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;
using Microsoft.MixedReality.Toolkit.Utilities;
using Microsoft.MixedReality.Toolkit.Input;
using Microsoft.MixedReality.Toolkit.UI;

public class ContinuousHandPub : MonoBehaviour
{
    // Hardcoded variables
    const int k_NumRobotJoints = 6;
    const float k_JointAssignmentWait = 0.1f;
    const float k_PoseAssignmentWait = 0.5f;

    // Variables required for ROS communication
    [SerializeField]
    string m_TopicName = "/wx250s/commands/hand_pose";

    // Variables required for ROS communication
    [SerializeField]
    // service advertised by interbotix_hand_joy
    string m_RosServiceName = "/wx250s/commands/gripper_position";
    // public string RosServiceName { get => m_RosServiceName; set => m_RosServiceName = value; }

    [SerializeField]
    GameObject m_RobotEndEffector;
    public GameObject RobotEndEffector { get => m_RobotEndEffector; set => m_RobotEndEffector = value; }

    [SerializeField]
    GameObject m_RobotBase;

    [SerializeField]
    GameObject m_WorkVolume;

    [SerializeField]
    GameObject m_GhostWorkVolume;

    [SerializeField]
    GameObject m_FeedbackHandler;

    [SerializeField]
    GameObject m_TutorialHandler;

    [SerializeField]
    GameObject m_EnableFollowButton;
    public GameObject EnableFollowButton { get => m_EnableFollowButton; set => m_EnableFollowButton = value; }

    // Articulation Bodies
    ArticulationBody[] m_JointArticulationBodies;

    // ROS Connector
    ROSConnection m_Ros;

    // Lock to keep the publisher from sending until successfully executed
    enum RobotState
    {
        Sleeping,
        Ready,
    }

    enum GripperState
    {
        Open,
        Closed
    }

    enum UserState
    {
        Hold,
        WaitForFollow,
        Follow
    }

    RobotState robot_state = RobotState.Sleeping;
    GripperState gripper_state = GripperState.Open;
    UserState user_state = UserState.Hold;
    
    // time used to check if user is still holding together thumb and index
    float time_next_check = 0.0f;
    float trigger_time = 0.5f;
    bool checked_twice = false;

    bool arm_execution_lock = false;
    bool gripper_execution_lock = false;
    bool gripper_closed = false;

    float waist_angle = 0.0f;
    float new_waist_angle = 0.0f;
    float work_area_distance = 0.5f;

    /// <summary>
    ///     Find all robot joints in Awake() and add them to the jointArticulationBodies array.
    ///     Find left and right finger joints and assign them to their respective articulation body objects.
    /// </summary>
    MixedRealityPose wrist_pose;
    MixedRealityPose index_pose;
    MixedRealityPose thumb_pose;
    void Start()
    {
        // Get ROS connection static instance
        m_Ros = ROSConnection.GetOrCreateInstance();
        m_Ros.RegisterRosService<GripperPositionRequest, GripperPositionResponse>(m_RosServiceName);
        m_Ros.RegisterRosService<SetBoolRequest, SetBoolResponse>("wx250s/commands/toggle_hand_follow");
        m_Ros.RegisterRosService<TriggerRequest, TriggerResponse>("wx250s/commands/go_home");
        m_Ros.RegisterRosService<TriggerRequest, TriggerResponse>("wx250s/commands/go_sleep");
        m_Ros.RegisterRosService<SetWaistAngleRequest, SetWaistAngleResponse>("wx250s/commands/set_waist_angle");
        m_Ros.RegisterPublisher<PoseMsg>(m_TopicName);
        arm_execution_lock = false;
        gripper_execution_lock = false; 
        gripper_closed = false;

        // new states
        robot_state = RobotState.Sleeping;
        gripper_state = GripperState.Open;
        user_state = UserState.Hold;
        
        InitWorkArea();
        // In case something was still running:
        StopContinuousHandFollow();
        GoSleep();
    }

    // Safeguard to keep the physical robot arm from crashing.
    void OnApplicationQuit()
    {
        StopContinuousHandFollow();
        GoSleep();
    }

    void InitWorkArea() {
        work_area_distance = m_WorkVolume.transform.localPosition.z;
        m_GhostWorkVolume.transform.position = m_WorkVolume.transform.position;
        m_GhostWorkVolume.transform.rotation = m_WorkVolume.transform.rotation;
        m_GhostWorkVolume.transform.localScale = m_WorkVolume.transform.localScale;
    }

    // ROBOT STATE SWITCHES
    private void SwitchToReady() {
        switch (robot_state) {
            case RobotState.Ready:
                break;
            case RobotState.Sleeping:
                ResetPos();
                break;
        }
    }

    public void SwitchToSleep() {
        switch (robot_state) {
            case RobotState.Sleeping:
                SwitchToHold();
                break;
            case RobotState.Ready:
                SwitchToHold();
                GoSleep();
                break;
        }
    }

    // USER STATE SWITCHES
    private void SwitchToWaitForFollow() {
        StopContinuousHandFollow();
        SwitchToReady();
        Update_Button_Content("IconClose", "Stop Following Hand");
        user_state = UserState.WaitForFollow;
    }

    private void SwitchToFollow() {
        // Make sure we are waiting to follow.
        if (user_state != UserState.WaitForFollow) {
            SwitchToWaitForFollow();
        }
        m_GhostWorkVolume.SetActive(false);
        StartContinuousHandFollow();
        user_state = UserState.Follow;
    }

    // Gets called internally and upon repositioning robot.
    public void SwitchToHold() {
        user_state = UserState.Hold;
        StopContinuousHandFollow();
        m_GhostWorkVolume.SetActive(false);
        Update_Button_Content("IconDone", "Start Following Hand");
    }

    // FUNCTIONS CALLED BY BUTTONS
    public void ToggleContinuousHandFollow() {
        switch (user_state) {
            case UserState.Hold:
                SwitchToWaitForFollow();
            break;
            case UserState.WaitForFollow:
            case UserState.Follow:
                SwitchToHold();
            break;
            default:
            break;
        } 
    }

    void StartContinuousHandFollow() 
    {   
        var request = new SetBoolRequest{
            data = true
        };
        m_Ros.SendServiceMessage<SetBoolResponse>("wx250s/commands/toggle_hand_follow", request, HandleFollowResponse);
    }

    public void StopContinuousHandFollow() 
    {
        // first check since the button calls start and stop functions
        var request = new SetBoolRequest{
            data = false
        };
        m_Ros.SendServiceMessage<SetBoolResponse>("wx250s/commands/toggle_hand_follow", request, HandleFollowResponse);
        if (robot_state != RobotState.Sleeping) {
            IndicateNotFollowing();
        }
    }

    void HandleFollowResponse(SetBoolResponse resp) {}

    void Update_Button_Content(string icon_name, string button_text)
    {   
        m_EnableFollowButton.GetComponent<ButtonConfigHelper>().SetQuadIconByName(icon_name);
        m_EnableFollowButton.GetComponent<ButtonConfigHelper>().MainLabelText = button_text;
    }

    public void ResetPos() 
    {   
        var request = new TriggerRequest();
        arm_execution_lock = true;
        m_Ros.SendServiceMessage<TriggerResponse>("wx250s/commands/go_home", request, HandleResetResponse);
    }

    void HandleResetResponse(TriggerResponse response)
    {
        arm_execution_lock = false;
        robot_state = RobotState.Ready;
    }

    private void GoSleep() 
    {   
        var request = new TriggerRequest();
        arm_execution_lock = true;
        m_Ros.SendServiceMessage<TriggerResponse>("wx250s/commands/go_sleep", request, HandleSleepResponse);
    }

    void HandleSleepResponse(TriggerResponse response)
    {
        arm_execution_lock = false;
        // arm is not ready to follow hand commands and should stop following them
        robot_state = RobotState.Sleeping;
        IndicateSleeping();
        waist_angle = 0.0f;
    }

    /// <summary>
    ///     Helper function that calls the feedback handler to indicate that robot is not following
    /// </summary>
    void IndicateNotFollowing() {
        m_FeedbackHandler.GetComponent<FeedbackHandler>().HandleUnityFeedback(3);
    }

    /// <summary>
    ///     Helper function that calls the feedback handler to indicate that robot is not following
    /// </summary>
    void IndicateSleeping() {
        m_FeedbackHandler.GetComponent<FeedbackHandler>().HandleUnityFeedback(4);
    }


    /// <summary>
    ///     Helper function that checks whether the hand is within the current working area
    /// </summary>
    bool CheckInsideWorkVolume(Vector3 point) {
        bool inside = m_WorkVolume.GetComponent<Renderer>().bounds.Contains(point);
        if (!inside) {
            IndicateNotFollowing();
        }
        return inside;
    }


    /// <summary>
    ///     Get hand input pose and publish it for the robot to follow.
    ///     Get index and thumb tip distance and issue a GripperPositionRequest if their closer than 
    ///     a threshold to close the gripper or wider than a threshold to open it again.
    /// </summary>
    void Update()
    {          
        if (user_state == UserState.Hold) {
            return;
        }

        // Defines the physical position of the gripper between the thumb proximal and index knuckle joint. The higher the closer to the index knuckle.
        float dist = 0.5f; 

        // Check if inside work volume - aka if we should do tracking:
        // We need to see the wrist, thumb proximal and index knuckle to compute an intuitive gripper pose for the user.
        if( HandJointUtils.TryGetJointPose(TrackedHandJoint.Wrist, Handedness.Right, out wrist_pose) &&
            HandJointUtils.TryGetJointPose(TrackedHandJoint.IndexKnuckle, Handedness.Right, out index_pose) &&
            HandJointUtils.TryGetJointPose(TrackedHandJoint.ThumbProximalJoint, Handedness.Right, out thumb_pose)) 
        {
            // This should give you the position between the thumb proximal and the index knuckle for ALL hand sizes
            Vector3 input_pos = thumb_pose.Position + (index_pose.Position - thumb_pose.Position) * dist;

            if (CheckInsideWorkVolume(input_pos)) 
            {               
                 // send arm request first since gripper request updates index and thumb pose
                if (!arm_execution_lock) {
                    if (user_state == UserState.WaitForFollow) {
                        SwitchToFollow();
                    }
                    SendArmRequest(input_pos);
                }
                
                if (!gripper_execution_lock) {
                    SendGripperRequest();
                }
            } else {
                if (user_state == UserState.Follow){
                    SwitchToWaitForFollow();
                }
                IndicateWorkArea(input_pos);
            }
        } else {
            // we didn't see hand
            IndicateNotFollowing();
            m_GhostWorkVolume.SetActive(false);
        }   
    }

    private void SendArmRequest(Vector3 input_pos) { 
        // We want the gripper to align with the thumb and index finger of the user, not the wrist plane
        // Quaternion of desired rotation in world coordinates:

        // Rotate the pose to match the orientation of the index knuckle and thumb proximal
        Quaternion offsetRot = wrist_pose.Rotation * Quaternion.Euler(25,0,65); 

        // relative rotation from end effector to wrist in world frame
        Quaternion input_rot = Quaternion.Inverse(m_RobotEndEffector.transform.rotation) * offsetRot;

        QuaternionMsg ros_input_rot = input_rot.To<FLU>();

        //geometry_msgs/Pose - requested end effector pose
        PoseMsg ee_pose = new PoseMsg{orientation = ros_input_rot};
    
        // get the vector from robot end effector to wrist:
        Vector3 bearing = (input_pos - m_RobotEndEffector.transform.position);
        // project coordinate frame into the one !rotated with robot and wrist angle
        float bearing_x = Vector3.Dot(bearing, m_WorkVolume.transform.right.normalized);
        float bearing_y = Vector3.Dot(bearing, m_WorkVolume.transform.up.normalized);
        float bearing_z =Vector3.Dot(bearing, m_WorkVolume.transform.forward.normalized);

        ee_pose.position = new Vector3(bearing_x, bearing_y, bearing_z).To<FLU>();

        m_Ros.Publish(m_TopicName, ee_pose);
    }

    private void SendGripperRequest() 
    {
        // Get the difference of the user's index finger and thumb as a metric if the gripper
        // should be closed or not. If one or both can't be tracked, the current gripper
        // position should be kept.
        if ((HandJointUtils.TryGetJointPose(TrackedHandJoint.IndexTip, Handedness.Right, out index_pose)) && 
            (HandJointUtils.TryGetJointPose(TrackedHandJoint.ThumbTip, Handedness.Right, out thumb_pose)))
        {
            var request = new GripperPositionRequest();
            Vector3 index_position = index_pose.Position;
            Vector3 thumb_position = thumb_pose.Position;

            // Check if the gripper is open and needs to be closed
            if (!gripper_closed && (Vector3.Distance(index_position, thumb_position) <= 0.03))
            {
                // Close the gripper.
                request.gripper_cmd = 0;
                gripper_closed = true;
                gripper_execution_lock = true;
                m_Ros.SendServiceMessage<GripperPositionResponse>(m_RosServiceName, request, HandleGripperResponse);
            } // Check if the gripper is closed and needs to be opened
            else if (gripper_closed && (Vector3.Distance(index_position, thumb_position) >= 0.05))
            {
                request.gripper_cmd = 1;
                gripper_closed = false;
                gripper_execution_lock = true;
                m_Ros.SendServiceMessage<GripperPositionResponse>(m_RosServiceName, request, HandleGripperResponse);
            }
        }
    }

    private void HandleGripperResponse(GripperPositionResponse response)
    {
        gripper_execution_lock = false;
        m_TutorialHandler.GetComponent<Tutorial>().ChangedGripper();
    }

    // we already know the wrist position and that we're outside of work area if this gets called
    // show where the new work area would be
    public void IndicateWorkArea(Vector3 input_pos) 
    {       
        // get the vector from robot waist to wrist:
        Vector3 bearing = (input_pos - m_RobotBase.transform.position);

        // project onto local coordinate frame
        float bearing_x = Vector3.Dot(bearing, m_RobotBase.transform.right.normalized);
        float bearing_z =Vector3.Dot(bearing, m_RobotBase.transform.forward.normalized);
        new_waist_angle = Mathf.Atan2(bearing_x, bearing_z); // angle relative to body
        float delta_angle = (new_waist_angle - waist_angle);
        if (Mathf.Abs(new_waist_angle - waist_angle) > 1.0f) {
            Vector3 new_trans = m_GhostWorkVolume.transform.localPosition;
            new_trans.x =  Mathf.Sin(new_waist_angle) * work_area_distance;
            new_trans.z =  Mathf.Cos(new_waist_angle) * work_area_distance;
            m_GhostWorkVolume.transform.localPosition = new_trans;
            m_GhostWorkVolume.transform.localRotation =  Quaternion.Euler(0.0f, new_waist_angle * Mathf.Rad2Deg , 0.0f);
            m_GhostWorkVolume.SetActive(true);

            // check if user is tapping 
            if ((HandJointUtils.TryGetJointPose(TrackedHandJoint.IndexTip, Handedness.Right, out index_pose)) && 
            (HandJointUtils.TryGetJointPose(TrackedHandJoint.ThumbTip, Handedness.Right, out thumb_pose)))
            {              
                if (Vector3.Distance(index_pose.Position, thumb_pose.Position) <= 0.03) {
                    if (Time.time > time_next_check) {
                        time_next_check = Time.time + trigger_time;
                        if (checked_twice) {
                            MoveWorkArea();
                            m_GhostWorkVolume.GetComponent<Renderer>().material.color = new Color32(0, 0, 0, 0);
                            checked_twice = false;
                        } else {
                            checked_twice = true;
                        }
                    } else {
                        // change display slowly as we get close to time next check
                        byte delta = (byte)Math.Floor((1 - (time_next_check - Time.time)/trigger_time) * 70);
                        m_GhostWorkVolume.GetComponent<Renderer>().material.color = new Color32(delta, delta, delta, 255);
                    }
                } else {
                    // check again next time
                    m_GhostWorkVolume.GetComponent<Renderer>().material.color = new Color32(0, 0, 0, 0);
                    time_next_check = Time.time;
                    checked_twice = false;
                }
            }
        } else {
            m_GhostWorkVolume.SetActive(false);
        }
    }

    public void MoveWorkArea() 
    {   
        // update visuals
        m_WorkVolume.transform.position = m_GhostWorkVolume.transform.position;
        m_WorkVolume.transform.rotation = m_GhostWorkVolume.transform.rotation;

        // set up request
        SetWaistAngleRequest request = new SetWaistAngleRequest();

        // transform from angle relative to z position to relative to x position
        request.waist_angle = (float)(-new_waist_angle);
        waist_angle = new_waist_angle;
        // stop following hand
        arm_execution_lock = true;
        gripper_execution_lock = true;
        m_Ros.SendServiceMessage<SetWaistAngleResponse>("wx250s/commands/set_waist_angle", request, HandleFlickOfTheWrist);
    }

    void HandleFlickOfTheWrist(SetWaistAngleResponse response) 
    {
        m_TutorialHandler.GetComponent<Tutorial>().MovedWorkArea();
        // start following hand again
        gripper_execution_lock = false;
        arm_execution_lock = false;
    }
}
