/* ----------------------------------------------------------------------------
 * This file was automatically generated by SWIG (http://www.swig.org).
 * Version 2.0.8
 *
 * Do not make changes to this file unless you know what you are doing--modify
 * the SWIG interface file instead.
 * ----------------------------------------------------------------------------- */


using System;
using System.Runtime.InteropServices;

public class aiAnimation : IDisposable {
  private HandleRef swigCPtr;
  protected bool swigCMemOwn;

  internal aiAnimation(IntPtr cPtr, bool cMemoryOwn) {
    swigCMemOwn = cMemoryOwn;
    swigCPtr = new HandleRef(this, cPtr);
  }

  internal static HandleRef getCPtr(aiAnimation obj) {
    return (obj == null) ? new HandleRef(null, IntPtr.Zero) : obj.swigCPtr;
  }

  ~aiAnimation() {
    Dispose();
  }

  public virtual void Dispose() {
    lock(this) {
      if (swigCPtr.Handle != IntPtr.Zero) {
        if (swigCMemOwn) {
          swigCMemOwn = false;
          AssimpPINVOKE.delete_aiAnimation(swigCPtr);
        }
        swigCPtr = new HandleRef(null, IntPtr.Zero);
      }
      GC.SuppressFinalize(this);
    }
  }

  public aiNodeAnimVector mChannels { get { return GetmChannels(); } }
  public aiMeshAnimVector mMeshChannels { get { return GetmMeshChannels(); } }

  public aiString mName {
    set {
      AssimpPINVOKE.aiAnimation_mName_set(swigCPtr, aiString.getCPtr(value));
    } 
    get {
      IntPtr cPtr = AssimpPINVOKE.aiAnimation_mName_get(swigCPtr);
      aiString ret = (cPtr == IntPtr.Zero) ? null : new aiString(cPtr, false);
      return ret;
    } 
  }

  public double mDuration {
    set {
      AssimpPINVOKE.aiAnimation_mDuration_set(swigCPtr, value);
    } 
    get {
      double ret = AssimpPINVOKE.aiAnimation_mDuration_get(swigCPtr);
      return ret;
    } 
  }

  public double mTicksPerSecond {
    set {
      AssimpPINVOKE.aiAnimation_mTicksPerSecond_set(swigCPtr, value);
    } 
    get {
      double ret = AssimpPINVOKE.aiAnimation_mTicksPerSecond_get(swigCPtr);
      return ret;
    } 
  }

  public uint mNumChannels {
    set {
      AssimpPINVOKE.aiAnimation_mNumChannels_set(swigCPtr, value);
    } 
    get {
      uint ret = AssimpPINVOKE.aiAnimation_mNumChannels_get(swigCPtr);
      return ret;
    } 
  }

  public uint mNumMeshChannels {
    set {
      AssimpPINVOKE.aiAnimation_mNumMeshChannels_set(swigCPtr, value);
    } 
    get {
      uint ret = AssimpPINVOKE.aiAnimation_mNumMeshChannels_get(swigCPtr);
      return ret;
    } 
  }

  public aiAnimation() : this(AssimpPINVOKE.new_aiAnimation(), true) {
  }

  private aiNodeAnimVector GetmChannels() {
    IntPtr cPtr = AssimpPINVOKE.aiAnimation_GetmChannels(swigCPtr);
    aiNodeAnimVector ret = (cPtr == IntPtr.Zero) ? null : new aiNodeAnimVector(cPtr, true);
    return ret;
  }

  private aiMeshAnimVector GetmMeshChannels() {
    IntPtr cPtr = AssimpPINVOKE.aiAnimation_GetmMeshChannels(swigCPtr);
    aiMeshAnimVector ret = (cPtr == IntPtr.Zero) ? null : new aiMeshAnimVector(cPtr, true);
    return ret;
  }

}
