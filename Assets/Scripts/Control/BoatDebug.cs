using OneBitLab.Services;
using System.Collections;
using System.Collections.Generic;
using UnityEditor;
using UnityEngine;

public class BoatDebug : MonoBehaviour
{
    // Start is called before the first frame update
    private Vector3 centerOfMass = Vector3.zero;
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        
    }
    private void OnDrawGizmos()
    {
#if UNITY_EDITOR
        centerOfMass = ResourceLocatorService.Instance.COM;
        Gizmos.color = Color.yellow;
        Vector3 worldCoM = transform.TransformPoint(centerOfMass);
        Gizmos.DrawSphere(worldCoM, 0.05f);
        Handles.Label(worldCoM, "CoM");
#endif
    }
}
