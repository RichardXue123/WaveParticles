using OneBitLab.Services;
using System.Collections;
using System.Collections.Generic;
using UnityEditor;
using UnityEngine;

public class BoatDebug : MonoBehaviour
{
    // Start is called before the first frame update
    private Vector3 centerOfMass = Vector3.zero;
    private Rigidbody targetRigidbody;
    void Start()
    {
        targetRigidbody = GetComponent<Rigidbody>();
    }

    // Update is called once per frame
    void Update()
    {
        
    }
    private void OnDrawGizmos()
    {
        targetRigidbody = GetComponent<Rigidbody>();
#if UNITY_EDITOR
        centerOfMass = ResourceLocatorService.Instance.COM;
        Gizmos.color = Color.yellow;
        Vector3 worldCoM = transform.TransformPoint(centerOfMass);
        Gizmos.DrawSphere(worldCoM, 0.05f);
        Handles.Label(worldCoM, "CoM");
        ResourceLocatorService.Instance.WorldCOM = worldCoM;
        ResourceLocatorService.Instance.inertiaTensor = targetRigidbody.inertiaTensor;
        ResourceLocatorService.Instance.intertiaRotation = targetRigidbody.inertiaTensorRotation;
        Debug.Log("Update Inertia:" + targetRigidbody.inertiaTensor);
        Debug.Log("Update intertiaRotation:" + targetRigidbody.inertiaTensorRotation.x + ","+ targetRigidbody.inertiaTensorRotation.y + "," + targetRigidbody.inertiaTensorRotation.z);
#endif
    }
}
