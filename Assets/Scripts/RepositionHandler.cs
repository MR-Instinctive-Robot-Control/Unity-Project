using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RepositionHandler : MonoBehaviour
{

    [SerializeField]
    GameObject m_Workspace;
    public GameObject Workspace { get => m_Workspace; set => m_Workspace = value; }

    [SerializeField]
    GameObject m_RobotBaseLink;
    public GameObject RobotBaseLink { get => m_RobotBaseLink; set => m_RobotBaseLink = value; }
    ArticulationBody robot_root;
    
    float init_distance_forward = 1.0f;
    float init_distance_down = 0.5f;

    // Start is called before the first frame update
    void Start()
    {
        robot_root = m_RobotBaseLink.GetComponent<ArticulationBody>();
    }

    public void startReposition() {
        // start off with the worcspace in front of the user oriented as he is
         m_Workspace.transform.position = 
                Camera.main.transform.position + 
                Camera.main.transform.forward * init_distance_forward +
                Camera.main.transform.up * -init_distance_down;
         m_Workspace.transform.rotation = Quaternion.Euler(0, Camera.main.transform.rotation.eulerAngles.y, 0);
    }

    public void applyReposition() {
        Vector3 position = m_Workspace.transform.position;
        Quaternion rotation = m_Workspace.transform.rotation;
        robot_root.TeleportRoot(position, rotation);
    }
}