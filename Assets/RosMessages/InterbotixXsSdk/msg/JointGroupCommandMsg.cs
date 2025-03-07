//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.InterbotixXsSdk
{
    [Serializable]
    public class JointGroupCommandMsg : Message
    {
        public const string k_RosMessageName = "interbotix_xs_sdk/JointGroupCommand";
        public override string RosMessageName => k_RosMessageName;

        //  Command the joints in the specified joint group. Note that the commands are processed differently based on the group's operating mode.
        //  For example, if a group's operating mode is set to 'position', the commands are interpreted as positions in radians
        public string name;
        //  Name of joint group
        public float[] cmd;
        //  List of joint commands; order is dictated by the index of each joint name for the given group in the 'groups' section of a 'motor_config' yaml file

        public JointGroupCommandMsg()
        {
            this.name = "";
            this.cmd = new float[0];
        }

        public JointGroupCommandMsg(string name, float[] cmd)
        {
            this.name = name;
            this.cmd = cmd;
        }

        public static JointGroupCommandMsg Deserialize(MessageDeserializer deserializer) => new JointGroupCommandMsg(deserializer);

        private JointGroupCommandMsg(MessageDeserializer deserializer)
        {
            deserializer.Read(out this.name);
            deserializer.Read(out this.cmd, sizeof(float), deserializer.ReadLength());
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.name);
            serializer.WriteLength(this.cmd);
            serializer.Write(this.cmd);
        }

        public override string ToString()
        {
            return "JointGroupCommandMsg: " +
            "\nname: " + name.ToString() +
            "\ncmd: " + System.String.Join(", ", cmd.ToList());
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
