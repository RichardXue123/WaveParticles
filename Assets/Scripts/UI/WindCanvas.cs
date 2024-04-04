using System;
using System.Collections;
using System.Collections.Generic;
using OneBitLab.FluidSim;
using Unity.Entities;
using UnityEngine;
using UnityEngine.UI;
using UnityEngine.XR.WSA;
using OneBitLab.Services;

public class WindCanvas : MonoBehaviour
{
    [SerializeField]
    private Text m_windSpeedText;

    private WaveSpawnSystem m_WaveSpawnSystem;
    // Start is called before the first frame update
    void Start()
    {
        m_WaveSpawnSystem = World.DefaultGameObjectInjectionWorld.GetOrCreateSystem<WaveSpawnSystem>();        
    }

    // // Update is called once per frame
    // void Update()
    // {
        
    // }
    public void UpdateWindSpeed( float windSpeed )
    {
        Debug.Log("OnEndEdit, text=" + windSpeed);
        // m_WaveSpawnSystem.m_windSpeed = windSpeed;
        SpectrumService.Instance.windSpeed=windSpeed;
        m_windSpeedText.text = ( (float)windSpeed ).ToString();
    }
}
