using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using RosMessageTypes.Geometry;
using RosMessageTypes.Std;

public class CubeSubscriber : MonoBehaviour
{
    private float size_cube = 0.07f;

    bool filtering = false;

    int id_marker_calibration;
    private float offsetx_marker_baseframe = 0.13f;
    private float offsety_marker_baseframe = 0.017f;

    bool cube1_created = false;
    bool cube2_created = false;
    bool cube3_created = false;
    bool cube4_created = false;

    bool is_cube1_detected = false;
    bool is_cube2_detected = false;
    bool is_cube3_detected = false;
    bool is_cube4_detected = false;

    [SerializeField]
    GameObject m_Robotbase;

    public GameObject cube1;
    public GameObject cube2;
    public GameObject cube3;
    public GameObject cube4;

    int length_moving_avg = 15;

    List<float> pos_x1 = new List<float>();
    List<float> pos_y1 = new List<float>();
    List<float> pos_z1 = new List<float>();

    List<float> pos_x2 = new List<float>();
    List<float> pos_y2 = new List<float>();
    List<float> pos_z2 = new List<float>();

    List<float> pos_x3 = new List<float>();
    List<float> pos_y3 = new List<float>();
    List<float> pos_z3 = new List<float>();

    List<float> pos_x4 = new List<float>();
    List<float> pos_y4 = new List<float>();
    List<float> pos_z4 = new List<float>();

    float prev_orient_x1 = 0.0f;
    float prev_orient_y1 = 0.0f;
    float prev_orient_z1 = 0.0f;
    float prev_orient_w1 = 0.0f;

    float prev_orient_x2 = 0.0f;
    float prev_orient_y2 = 0.0f;
    float prev_orient_z2 = 0.0f;
    float prev_orient_w2 = 0.0f;

    float prev_orient_x3 = 0.0f;
    float prev_orient_y3 = 0.0f;
    float prev_orient_z3 = 0.0f;
    float prev_orient_w3 = 0.0f;

    float prev_orient_x4 = 0.0f;
    float prev_orient_y4 = 0.0f;
    float prev_orient_z4 = 0.0f;
    float prev_orient_w4 = 0.0f;




    // ROS Connector
    ROSConnection m_Ros;

    void Start()
    {

        m_Ros = ROSConnection.GetOrCreateInstance();

        m_Ros.Subscribe<Int32Msg>("/publisher_cube_position/id_marker_calibration", GetCalibrationID);

        m_Ros.Subscribe<BoolMsg>("/publisher_cube_position/is_cube1_detected", Detection1Callback);
        m_Ros.Subscribe<PoseMsg>("/publisher_cube_position/cube1_pose", Pose1Callback);

        m_Ros.Subscribe<BoolMsg>("/publisher_cube_position/is_cube2_detected", Detection2Callback);
        m_Ros.Subscribe<PoseMsg>("/publisher_cube_position/cube2_pose", Pose2Callback);

        m_Ros.Subscribe<BoolMsg>("/publisher_cube_position/is_cube3_detected", Detection3Callback);
        m_Ros.Subscribe<PoseMsg>("/publisher_cube_position/cube3_pose", Pose3Callback);

        m_Ros.Subscribe<BoolMsg>("/publisher_cube_position/is_cube4_detected", Detection4Callback);
        m_Ros.Subscribe<PoseMsg>("/publisher_cube_position/cube4_pose", Pose4Callback);


        Debug.Log("Subscriber initialized...");
    }


    void GetCalibrationID(Int32Msg msg)
    {
        id_marker_calibration = (int)msg.data;
    }

    void Detection1Callback(BoolMsg msg)
    {
        is_cube1_detected = (bool)msg.data;
    }

    void Detection2Callback(BoolMsg msg)
    {
        is_cube2_detected = (bool)msg.data;
    }

    void Detection3Callback(BoolMsg msg)
    {
        is_cube3_detected = (bool)msg.data;
    }

    void Detection4Callback(BoolMsg msg)
    {
        is_cube4_detected = (bool)msg.data;
    }




    void Pose1Callback(PoseMsg msg)
    {

        if (is_cube1_detected)
        {
            if (!(((float)msg.position.x == 0.0f) && ((float)msg.position.y == 0.0f) && ((float)msg.position.z == 0.0f)))
            {
                if (filtering)
                {
                    pos_x1.Add((float)msg.position.x);
                    pos_y1.Add((float)msg.position.y);
                    pos_z1.Add((float)msg.position.z);


                    if (pos_x1.Count == length_moving_avg)
                    {
                        pos_x1.RemoveAt(0);
                        pos_y1.RemoveAt(0);
                        pos_z1.RemoveAt(0);

                    }

                    float avg_x = pos_x1.Average();
                    float avg_y = pos_y1.Average();
                    float avg_z = pos_z1.Average();

                    cube1.transform.localPosition = new Vector3(avg_x, avg_y, avg_z);

                    Quaternion orientation = new Quaternion();
                    orientation.x = (float)msg.orientation.x;
                    orientation.y = (float)msg.orientation.y;
                    orientation.z = (float)msg.orientation.z;
                    orientation.w = (float)msg.orientation.w;

                    float roll = orientation.eulerAngles.x;
                    float pitch = orientation.eulerAngles.y;
                    float yaw = orientation.eulerAngles.z;


                    if ((Math.Abs(roll - prev_orient_x1) > 0.01) || (Math.Abs(pitch - prev_orient_y1) > 0.01) || (Math.Abs(yaw - prev_orient_z1) > 0.01))
                    {
                        cube1.transform.localRotation = orientation;

                        prev_orient_x1 = roll;
                        prev_orient_y1 = pitch;
                        prev_orient_z1 = yaw;
                    }
                    
                }
                else
                {
                    if (id_marker_calibration == 49)
                    {
                        cube1.transform.localPosition = new Vector3((float)msg.position.x - offsetx_marker_baseframe, (float)msg.position.y + offsety_marker_baseframe, (float)msg.position.z);
                    }
                    else if (id_marker_calibration == 48)
                    {
                        cube1.transform.localPosition = new Vector3((float)msg.position.x + offsetx_marker_baseframe, (float)msg.position.y + offsety_marker_baseframe, (float)msg.position.z);
                    }
                    

                    Quaternion orientation = new Quaternion();
                    orientation.x = (float)msg.orientation.x;
                    orientation.y = (float)msg.orientation.y;
                    orientation.z = (float)msg.orientation.z;
                    orientation.w = (float)msg.orientation.w;

                    cube1.transform.localRotation = orientation;
                }
            }           
        }

    }



    void Pose2Callback(PoseMsg msg)
    {

        if (is_cube2_detected)
        {
            if (!(((float)msg.position.x == 0.0f) && ((float)msg.position.y == 0.0f) && ((float)msg.position.z == 0.0f)))
            {
                if (filtering)
                {
                    pos_x2.Add((float)msg.position.x);
                    pos_y2.Add((float)msg.position.y);
                    pos_z2.Add((float)msg.position.z);


                    if (pos_x2.Count == length_moving_avg)
                    {
                        pos_x2.RemoveAt(0);
                        pos_y2.RemoveAt(0);
                        pos_z2.RemoveAt(0);

                    }

                    float avg_x = pos_x2.Average();
                    float avg_y = pos_y2.Average();
                    float avg_z = pos_z2.Average();

                    cube2.transform.localPosition = new Vector3(avg_x, avg_y, avg_z);

                    Quaternion orientation = new Quaternion();
                    orientation.x = (float)msg.orientation.x;
                    orientation.y = (float)msg.orientation.y;
                    orientation.z = (float)msg.orientation.z;
                    orientation.w = (float)msg.orientation.w;

                    float roll = orientation.eulerAngles.x;
                    float pitch = orientation.eulerAngles.y;
                    float yaw = orientation.eulerAngles.z;


                    if ((Math.Abs(roll - prev_orient_x2) > 0.01) || (Math.Abs(pitch - prev_orient_y2) > 0.01) || (Math.Abs(yaw - prev_orient_z2) > 0.01))
                    {
                        cube2.transform.localRotation = orientation;

                        prev_orient_x2 = roll;
                        prev_orient_y2 = pitch;
                        prev_orient_z2 = yaw;
                    }

                }
                else
                {
                    if (id_marker_calibration == 49)
                    {
                        cube2.transform.localPosition = new Vector3((float)msg.position.x - offsetx_marker_baseframe, (float)msg.position.y + offsety_marker_baseframe, (float)msg.position.z);
                    }
                    else if (id_marker_calibration == 48)
                    {
                        cube2.transform.localPosition = new Vector3((float)msg.position.x + offsetx_marker_baseframe, (float)msg.position.y + offsety_marker_baseframe, (float)msg.position.z);
                    }

                    Quaternion orientation = new Quaternion();
                    orientation.x = (float)msg.orientation.x;
                    orientation.y = (float)msg.orientation.y;
                    orientation.z = (float)msg.orientation.z;
                    orientation.w = (float)msg.orientation.w;

                    cube2.transform.localRotation = orientation;        

                }
            }
        }
    }



    void Pose3Callback(PoseMsg msg)
    {

        if (is_cube3_detected)
        {
            if (!(((float)msg.position.x == 0.0f) && ((float)msg.position.y == 0.0f) && ((float)msg.position.z == 0.0f)))
            {
                if (filtering)
                {
                    pos_x3.Add((float)msg.position.x);
                    pos_y3.Add((float)msg.position.y);
                    pos_z3.Add((float)msg.position.z);


                    if (pos_x3.Count == length_moving_avg)
                    {
                        pos_x3.RemoveAt(0);
                        pos_y3.RemoveAt(0);
                        pos_z3.RemoveAt(0);

                    }

                    float avg_x = pos_x3.Average();
                    float avg_y = pos_y3.Average();
                    float avg_z = pos_z3.Average();

                    cube3.transform.localPosition = new Vector3(avg_x, avg_y, avg_z);

                    Quaternion orientation = new Quaternion();
                    orientation.x = (float)msg.orientation.x;
                    orientation.y = (float)msg.orientation.y;
                    orientation.z = (float)msg.orientation.z;
                    orientation.w = (float)msg.orientation.w;

                    float roll = orientation.eulerAngles.x;
                    float pitch = orientation.eulerAngles.y;
                    float yaw = orientation.eulerAngles.z;


                    if ((Math.Abs(roll - prev_orient_x3) > 0.01) || (Math.Abs(pitch - prev_orient_y3) > 0.01) || (Math.Abs(yaw - prev_orient_z3) > 0.01))
                    {
                        cube3.transform.localRotation = orientation;

                        prev_orient_x3 = roll;
                        prev_orient_y3 = pitch;
                        prev_orient_z3 = yaw;
                    }

                }
                else
                {
                    if (id_marker_calibration == 49)
                    {
                        cube3.transform.localPosition = new Vector3((float)msg.position.x - offsetx_marker_baseframe, (float)msg.position.y + offsety_marker_baseframe, (float)msg.position.z);
                    }
                    else if (id_marker_calibration == 48)
                    {
                        cube3.transform.localPosition = new Vector3((float)msg.position.x + offsetx_marker_baseframe, (float)msg.position.y + offsety_marker_baseframe, (float)msg.position.z);
                    }

                    Quaternion orientation = new Quaternion();
                    orientation.x = (float)msg.orientation.x;
                    orientation.y = (float)msg.orientation.y;
                    orientation.z = (float)msg.orientation.z;
                    orientation.w = (float)msg.orientation.w;

                    cube3.transform.localRotation = orientation;
                }
            }
        }
    }



    void Pose4Callback(PoseMsg msg)
    {
        if (is_cube4_detected)
        {
            if (!(((float)msg.position.x == 0.0f) && ((float)msg.position.y == 0.0f) && ((float)msg.position.z == 0.0f)))
            {
                if (filtering)
                {
                    pos_x4.Add((float)msg.position.x);
                    pos_y4.Add((float)msg.position.y);
                    pos_z4.Add((float)msg.position.z);


                    if (pos_x4.Count == length_moving_avg)
                    {
                        pos_x4.RemoveAt(0);
                        pos_y4.RemoveAt(0);
                        pos_z4.RemoveAt(0);

                    }

                    float avg_x = pos_x4.Average();
                    float avg_y = pos_y4.Average();
                    float avg_z = pos_z4.Average();

                    cube4.transform.localPosition = new Vector3(avg_x, avg_y, avg_z);

                    Quaternion orientation = new Quaternion();
                    orientation.x = (float)msg.orientation.x;
                    orientation.y = (float)msg.orientation.y;
                    orientation.z = (float)msg.orientation.z;
                    orientation.w = (float)msg.orientation.w;

                    float roll = orientation.eulerAngles.x;
                    float pitch = orientation.eulerAngles.y;
                    float yaw = orientation.eulerAngles.z;


                    if ((Math.Abs(roll - prev_orient_x4) > 0.01) || (Math.Abs(pitch - prev_orient_y4) > 0.01) || (Math.Abs(yaw - prev_orient_z4) > 0.01))
                    {
                        cube4.transform.localRotation = orientation;

                        prev_orient_x4 = roll;
                        prev_orient_y4 = pitch;
                        prev_orient_z4 = yaw;
                    }

                }
                else
                {
                    if (id_marker_calibration == 49)
                    {
                        cube4.transform.localPosition = new Vector3((float)msg.position.x - offsetx_marker_baseframe, (float)msg.position.y + offsety_marker_baseframe, (float)msg.position.z);
                    }
                    else if (id_marker_calibration == 48)
                    {
                        cube4.transform.localPosition = new Vector3((float)msg.position.x + offsetx_marker_baseframe, (float)msg.position.y + offsety_marker_baseframe, (float)msg.position.z);
                    }

                    Quaternion orientation = new Quaternion();
                    orientation.x = (float)msg.orientation.x;
                    orientation.y = (float)msg.orientation.y;
                    orientation.z = (float)msg.orientation.z;
                    orientation.w = (float)msg.orientation.w;

                    cube4.transform.localRotation = orientation;
                }
            }
        }
    }



}
