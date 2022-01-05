using System.Collections;
using System.Collections.Generic;
using RosMessageTypes.Std;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

public class FeedbackHandler : MonoBehaviour
{
    [SerializeField]
    GameObject m_FeedbackIndicator;

    Color32 green = new Color32(18, 255, 94, 255);
    Color32 red =  new Color32(255, 35, 18, 255);
    Color32 dark_grey = new Color32(15, 15, 15, 255);
    Color32 awake = new Color32(209, 209, 209, 255);

    void Start()
    {
        ROSConnection.GetOrCreateInstance().Subscribe<BoolMsg>("/wx250s/feedback/hand_pose_feasible", HandleFeedbackMsg);
        awake = m_FeedbackIndicator.GetComponent<Renderer>().material.color;
    }

    void HandleFeedbackMsg(BoolMsg msg)
    {
        if (msg.data == true) {
            m_FeedbackIndicator.GetComponent<Renderer>().material.color = green;
        } else {
            m_FeedbackIndicator.GetComponent<Renderer>().material.color = red;
        }
    }

    public void HandleUnityFeedback(int status) {
        switch (status) {
            // 1 --> green
            case 1:
                m_FeedbackIndicator.GetComponent<Renderer>().material.color = green;
                break;
            // 2 --> red
            case 2:
                m_FeedbackIndicator.GetComponent<Renderer>().material.color = red;
                break;
            // 3 --> corresponds to awake
            case 3:
                m_FeedbackIndicator.GetComponent<Renderer>().material.color = awake;
                break;
            // 4 --> sleeping, black
            case 4:
                m_FeedbackIndicator.GetComponent<Renderer>().material.color = dark_grey;
                break;
            default:
                m_FeedbackIndicator.GetComponent<Renderer>().material.color = dark_grey;
                break;
        }
    }
}
