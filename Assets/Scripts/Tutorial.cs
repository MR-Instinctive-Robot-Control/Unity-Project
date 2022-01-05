using System;
using System.Collections;
using System.Linq;
using UnityEngine;
using TMPro;

public class Tutorial : MonoBehaviour
{
    // actually one less then elements in content
    const int kTutorialSteps = 5; 
    bool ongoing = true;
    int step = 0;

    [SerializeField]
    GameObject m_TutorialDisplay;

    [SerializeField]
    GameObject m_TutorialTitle;

    [SerializeField]
    GameObject m_TutorialContent;

    [SerializeField]
    GameObject m_ButtonNext;

    [SerializeField]
    GameObject m_ButtonSkip;

    [SerializeField]
    GameObject m_ButtonFinish;


    string[][] content = new string[][]
    {
    // title and content
    // 0
    new string[] {"Getting Started - Reposition the Hologram", 
                    "Let's get you started by positioning the robot in a comfortable space.\r\n" + 
                    "Click the \"Repositon Robot Button\" to start."},
    // 1
    new string[] {"Getting Started - Confirm Hologram Position", 
                    "Drag the semitranslucent robot hologram by grabing it with your hand(s), " +
                    "then let go if you're happy with the position and rotation.\r\n" +  
                    "Finally, confirm the pose by clicking on the button \"Click Here to Set new Workspace\"."},
    // 2
    new string[] {"Getting Started - Robot Interaction Menu", 
                    "Now bring up the robot interaction menu by looking at the palm of your left hand.\r\n" +
                    "<b> Start Following Hand</b>: Wakes up the robot to its default position and starts following your right hand.\r\n" +
                    "<b> Reset Position</b>: Resets the robot to the default position in the current work volume.\r\n" +
                    "<b> Go Sleep</b>: Returns the robot to its sleep position.\r\n" +
                    "\r\nMake sure that your right hand is in the translucent green cube and that you are standing behind the robot, " +
                    "that the real robot is safely positioned, and then click \"Start Following Hand\" to continue."},
    // 3
    new string[] {"Getting Started - Controlling the Arm and Gripper", 
                    "The robot arm should now follow your right hand and the gripper should be positioned ontop of your thumb and index finger.\r\n" +
                    "The green box that your hand is in at the moment marks the current working volume in which the robot is able to "+
                    "smoothly follow your hand. The robot will only listen to your hand commands if your hand is inside this area, and will hold it's " +
                    "pose and gripper state if you leave the area.\r\n" +
                    "You can open and close the gripper by bringing your thumb and index finger together or moving them apart. \r\n" +
                    "Try it to continue!"},
    // 4
    new string[] {"Getting Started - Changing the Working Volume",
                    "If hand tracking is enabled and you move your right hand far enough away from this working volume you will see a translucent new working volume, " +
                    "which can be placed anywhere around the robot where it is able to move to.\r\n" +
                    "Holding together your right index and thumb for 0.5s selects this as new working volume, and the robot will move there and start following your hand again.\r\n" +
                    "Select a new working volume to continue."},
    // 5
    new string[] {"Getting Started - Status Indicator", 
                    "On the top of the robot hologram you find a status indicator. When you move your hand inside of the outlined "+
                    "working volume the indicator changes color:\r\n" +
                    "<b> Green</b>: Robot is able to follow your hand\r\n" +
                    "<b> Red</b>: Robot cannot follow your hand\r\n" +
                    "<b> White</b>: Robot is not following your hand, potentially because your right hand is not inside the Work Volume\r\n"},
    };
    

    // Start is called before the first frame update
    void Start()
    {
        ongoing = true;
        step = 0;
        UpdateContent();
    }

    // callable functions
    public void ButtonSkipTutorial() 
    {
        ongoing = false;
        UpdateContent();
    }

    public void RestartTutorial() {
        Start();
    }

    public void NextStep() 
    {
        if (step < kTutorialSteps) {
            step += 1;
        } else {
            ongoing = false;
        }
        UpdateContent();        
    }

    // check that actions are performed during the right step
    public void ClickReposition() {
        if (step == 0) {
            NextStep();
        } 
    }

    public void ConfirmReposition() {
        if (step == 1) {
            NextStep();
        } 
    }

    public void StartFollow() {
        if (step == 2) {
            NextStep();
        } 
    }

    public void ChangedGripper() {
        if (step == 3) {
            NextStep();
        } 
    }

    public void MovedWorkArea() {
        if (step == 4) {
            NextStep();
        } 
    }

    // inernal update functions
    private void UpdateContent() 
    {
        // check if tutorial is being shown
        bool display_active = m_TutorialDisplay.activeSelf;

        if (!ongoing && display_active) {
            m_TutorialDisplay.SetActive(false);
        } else {
            if (!display_active) {
                m_TutorialDisplay.SetActive(true);
            }
            if (step < kTutorialSteps) {
                m_ButtonFinish.SetActive(false);
                m_ButtonSkip.SetActive(true);
                m_ButtonNext.SetActive(true);
            } else {
                // last step, show finish tutorial button
                m_ButtonFinish.SetActive(true);
                m_ButtonSkip.SetActive(false);
                m_ButtonNext.SetActive(false);
            }
            m_TutorialTitle.GetComponent<TextMeshPro>().SetText(content[step][0]);
            m_TutorialContent.GetComponent<TextMeshPro>().SetText(content[step][1]);
        }

    }



    // private void UpdateButtonContent(string icon_name, string button_text)
    // {   
    //     m_EnableFollowButton.GetComponent<ButtonConfigHelper>().SetQuadIconByName(icon_name);
    //     m_EnableFollowButton.GetComponent<ButtonConfigHelper>().MainLabelText = button_text;
    // }

    // private void UpdateTitle(string icon_name, string button_text)
    // {   
    //     m_EnableFollowButton.GetComponent<ButtonConfigHelper>().SetQuadIconByName(icon_name);
    //     m_EnableFollowButton.GetComponent<ButtonConfigHelper>().MainLabelText = button_text;
    // }
}
