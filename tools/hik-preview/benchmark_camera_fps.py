import sys, time, json
from pathlib import Path
from ctypes import *
from preview import *

cam=MvCamera(); opened=created=grabbing=False; saved={}; init=False
try:
 check(MvCamera.MV_CC_Initialize(),'initialize');init=True
 ds=MV_CC_DEVICE_INFO_LIST();check(MvCamera.MV_CC_EnumDevices(MV_USB_DEVICE,ds),'enumerate')
 if ds.nDeviceNum != 1: raise RuntimeError(f'USB camera count: {ds.nDeviceNum}')
 check(cam.MV_CC_CreateHandle(cast(ds.pDeviceInfo[0],POINTER(MV_CC_DEVICE_INFO)).contents),'handle');created=True
 check(cam.MV_CC_OpenDevice(MV_ACCESS_Exclusive,0),'open camera; close other software');opened=True
 for key in ['TriggerMode','ExposureAuto','GainAuto','TriggerSource']:
  v=MVCC_ENUMVALUE();rc=cam.MV_CC_GetEnumValue(key,v);print('enum',key,rc,v.nCurValue,flush=True)
  if rc==0:
   saved[key]=v.nCurValue
   if key != 'TriggerSource':check(cam.MV_CC_SetEnumValue(key,0),key)
 for key in ['ExposureTime','Gain','AcquisitionFrameRate','ResultingFrameRate']:
  v=MVCC_FLOATVALUE();rc=cam.MV_CC_GetFloatValue(key,v);print('float',key,rc,v.fCurValue,v.fMin,v.fMax,flush=True)
  if rc==0 and key in ('ExposureTime','Gain','AcquisitionFrameRate'):saved[key]=v.fCurValue
 b=c_bool();rc=cam.MV_CC_GetBoolValue('AcquisitionFrameRateEnable',b);print('bool AcquisitionFrameRateEnable',rc,b.value,flush=True)
 if rc==0:
  saved['AcquisitionFrameRateEnable']=b.value
  print('disable frame limit',cam.MV_CC_SetBoolValue('AcquisitionFrameRateEnable',False),flush=True)
 for key in ['Width','Height','PayloadSize']:
  v=MVCC_INTVALUE_EX();rc=cam.MV_CC_GetIntValueEx(key,v);print('int',key,rc,v.nCurValue,flush=True)
 check(cam.MV_CC_StartGrabbing(),'start');grabbing=True
 for requested in [50000,100000,150000]:
  exp=MVCC_FLOATVALUE();check(cam.MV_CC_GetFloatValue('ExposureTime',exp),'range')
  val=max(exp.fMin,min(exp.fMax,requested));check(cam.MV_CC_SetFloatValue('ExposureTime',val),'exposure')
  check(cam.MV_CC_ClearImageBuffer(),'clear');time.sleep(.3);check(cam.MV_CC_ClearImageBuffer(),'clear')
  n=0;first=None;last=None;frames=[];t0=time.perf_counter(); deadline=t0+5
  while time.perf_counter()<deadline:
   f=MV_FRAME_OUT();check(cam.MV_CC_GetImageBuffer(f,3000),'buffer');t=time.perf_counter();n+=1
   if first is None:first=t
   last=t;frames.append(int(f.stFrameInfo.nFrameNum));cam.MV_CC_FreeImageBuffer(f)
  elapsed=last-first; rate=(n-1)/elapsed;rv=MVCC_FLOATVALUE();rc=cam.MV_CC_GetFloatValue('ResultingFrameRate',rv)
  missing=sum(max(0,(b-a)-1) for a,b in zip(frames,frames[1:]) if b>=a)
  print(json.dumps({'exposure_us':val,'frames':n,'seconds':round(elapsed,4),'fps':round(rate,3),'reported_fps':rv.fCurValue if rc==0 else None,'lost_frame_numbers':missing,'start_frame':frames[0],'end_frame':frames[-1]}),flush=True)
 check(cam.MV_CC_StopGrabbing(),'stop for trigger');grabbing=False
 check(cam.MV_CC_SetEnumValue('TriggerMode',MV_TRIGGER_MODE_ON),'trigger mode')
 check(cam.MV_CC_SetEnumValue('TriggerSource',MV_TRIGGER_SOURCE_SOFTWARE),'software trigger')
 check(cam.MV_CC_StartGrabbing(),'trigger start');grabbing=True
 for val in [50000,100000,150000]:
  check(cam.MV_CC_SetFloatValue('ExposureTime',float(val)),'trigger exposure')
  frames=[];times=[];t0=time.perf_counter();deadline=t0+5
  while time.perf_counter()<deadline:
   check(cam.MV_CC_SetCommandValue('TriggerSoftware'),'trigger')
   f=MV_FRAME_OUT();check(cam.MV_CC_GetImageBuffer(f,3000),'trigger frame');times.append(time.perf_counter());frames.append(int(f.stFrameInfo.nFrameNum));cam.MV_CC_FreeImageBuffer(f)
  print(json.dumps({'mode':'software_trigger','exposure_us':val,'frames':len(frames),'fps':round((len(frames)-1)/(times[-1]-times[0]),3),'lost_frame_numbers':sum(max(0,b-a-1) for a,b in zip(frames,frames[1:]) if b>=a)}),flush=True)

finally:
 if grabbing:cam.MV_CC_StopGrabbing()
 if opened:
  for key in ['AcquisitionFrameRate','ExposureTime','Gain']:
   if key in saved:cam.MV_CC_SetFloatValue(key,saved[key])
  if 'AcquisitionFrameRateEnable' in saved:cam.MV_CC_SetBoolValue('AcquisitionFrameRateEnable',saved['AcquisitionFrameRateEnable'])
  for key in ['ExposureAuto','GainAuto','TriggerSource','TriggerMode']:
   if key in saved:cam.MV_CC_SetEnumValue(key,saved[key])
  cam.MV_CC_CloseDevice()
 if created:cam.MV_CC_DestroyHandle()
 if init:MvCamera.MV_CC_Finalize()
