using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using UnityEngine;

public class TestSS : MonoBehaviour
{
    void Update()
    {
        if (Input.GetKeyDown(KeyCode.Space))
        {
            TakeSS();
        }
    }

    void TakeSS()
    {
        string path = @"C:\Users\Mazlum\Desktop\ısıl rapor";
        int i = 0;
        while (File.Exists(path + @"\SSTest" + i + ".png"))
        {
            i++;
        }
        ScreenCapture.CaptureScreenshot(path + @"\SSTest" + i + ".png");
    }
}
