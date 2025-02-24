//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.InterbotixHandJoy
{
    [Serializable]
    public class GripperPositionRequest : Message
    {
        public const string k_RosMessageName = "interbotix_hand_joy/GripperPosition";
        public override string RosMessageName => k_RosMessageName;

        //  1 for opening, 0 for closing the gripper
        public sbyte gripper_cmd;

        public GripperPositionRequest()
        {
            this.gripper_cmd = 0;
        }

        public GripperPositionRequest(sbyte gripper_cmd)
        {
            this.gripper_cmd = gripper_cmd;
        }

        public static GripperPositionRequest Deserialize(MessageDeserializer deserializer) => new GripperPositionRequest(deserializer);

        private GripperPositionRequest(MessageDeserializer deserializer)
        {
            deserializer.Read(out this.gripper_cmd);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.gripper_cmd);
        }

        public override string ToString()
        {
            return "GripperPositionRequest: " +
            "\ngripper_cmd: " + gripper_cmd.ToString();
        }

#if UNITY_EDITOR
        [UnityEditor.InitializeOnLoadMethod]
#else
        [UnityEngine.RuntimeInitializeOnLoadMethod]
#endif
        public static void Register()
        {
            MessageRegistry.Register(k_RosMessageName, Deserialize);
        }
    }
}
