using UnityEngine;
using UnityEngine.SceneManagement;

public class SceneCtrl : MonoBehaviour
{
   public void GoMainScene()
    {
        SceneManager.LoadScene("MainScene");
    }
}
