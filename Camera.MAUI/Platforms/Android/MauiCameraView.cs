#define TAP_TO_FOCUS_CONTINIOUS

using Android.Content;
using Android.Widget;
using Java.Util.Concurrent;
using Android.Graphics;
using CameraCharacteristics = Android.Hardware.Camera2.CameraCharacteristics;
using Android.Hardware.Camera2;
using Android.Media;
using Android.Views;
using Android.Util;
using Android.Hardware.Camera2.Params;
using Size = Android.Util.Size;
using Class = Java.Lang.Class;
using Rect = Android.Graphics.Rect;
using SizeF = Android.Util.SizeF;
using Android.Runtime;
using Android.OS;
using Android.Renderscripts;
using RectF = Android.Graphics.RectF;
using Android.Content.Res;
using Java.Util.Functions;
using Microsoft.Extensions.Logging;

namespace Camera.MAUI.Platforms.Android;

internal class MauiCameraView: GridLayout
{
    private readonly CameraView cameraView;
    private IExecutorService executorService;
    private bool started = false;
    private int frames = 0;
    private bool initiated = false;
    private bool snapping = false;
    private bool recording = false;
    private readonly Context context;

    private readonly TextureView textureView;
    public CameraCaptureSession previewSession;
    public MediaRecorder mediaRecorder;
    private CaptureRequest.Builder previewBuilder;
    private CameraDevice cameraDevice;
    private object cameraDeviceLock = new object();
    private readonly MyCameraStateCallback stateListener;
    private Size videoSize;
    private CameraManager cameraManager;
    private AudioManager audioManager;
    private readonly System.Timers.Timer timer;
    private readonly SparseIntArray ORIENTATIONS = new();
    private readonly SparseIntArray ORIENTATIONSFRONT = new();
    private CameraCharacteristics camChars;
    private PreviewCaptureStateCallback sessionCallback;
    private byte[] capturePhoto = null;
    private bool captureDone = false;
    private readonly ImageAvailableListener photoListener;
    private HandlerThread backgroundThread;
    private Handler backgroundHandler;
    private ImageReader imgReader;


    const int _useControlZoomRatio_ApiLevel = 30;
    //const int _useControlZoomRatio_ApiLevel = 100;
    readonly ILogger _logger;
    Action<string> _logger_LogTrace = null;

    public MauiCameraView(Context context, CameraView cameraView) : base(context)
    {
        _logger = IPlatformApplication.Current.Services.GetService<ILogger<MauiCameraView>>();
        if(_logger.IsEnabled(LogLevel.Trace))
        {
            _logger_LogTrace = (str) => _logger.LogTrace(str);
        }


        this.context = context;
        this.cameraView = cameraView;

        textureView = new(context);
        timer = new(33.3);
        timer.Elapsed += Timer_Elapsed;
        stateListener = new MyCameraStateCallback(this, _logger, _logger_LogTrace);
        photoListener = new ImageAvailableListener(this);
        AddView(textureView);
        ORIENTATIONS.Append((int)SurfaceOrientation.Rotation0, 90);
        ORIENTATIONS.Append((int)SurfaceOrientation.Rotation90, 0);
        ORIENTATIONS.Append((int)SurfaceOrientation.Rotation180, 270);
        ORIENTATIONS.Append((int)SurfaceOrientation.Rotation270, 180);
        ORIENTATIONSFRONT.Append((int)SurfaceOrientation.Rotation0, 270);
        ORIENTATIONSFRONT.Append((int)SurfaceOrientation.Rotation90, 0);
        ORIENTATIONSFRONT.Append((int)SurfaceOrientation.Rotation180, 90);
        ORIENTATIONSFRONT.Append((int)SurfaceOrientation.Rotation270, 180);
        InitDevices();
    }

    private void InitDevices()
    {
        if (!initiated && cameraView != null)
        {
            cameraManager = (CameraManager)context.GetSystemService(Context.CameraService);
            audioManager = (AudioManager)context.GetSystemService(Context.AudioService);
            cameraView.Cameras.Clear();
            foreach (var id in cameraManager.GetCameraIdList())
            {
                var cameraInfo = new CameraInfo { DeviceId = id, MinZoomFactor = 1f };
                var chars = cameraManager.GetCameraCharacteristics(id);
                if ((int)(chars.Get(CameraCharacteristics.LensFacing) as Java.Lang.Number) == (int)LensFacing.Back)
                {
                    cameraInfo.Name = "Back Camera";
                    cameraInfo.Position = CameraPosition.Back;
                }
                else if ((int)(chars.Get(CameraCharacteristics.LensFacing) as Java.Lang.Number) == (int)LensFacing.Front)
                {
                    cameraInfo.Name = "Front Camera";
                    cameraInfo.Position = CameraPosition.Front;
                }
                else
                {
                    cameraInfo.Name = "Camera " + id;
                    cameraInfo.Position = CameraPosition.Unknow;
                }
                cameraInfo.MaxZoomFactor = (float)(chars.Get(CameraCharacteristics.ScalerAvailableMaxDigitalZoom) as Java.Lang.Number);
                cameraInfo.HasFlashUnit = (bool)(chars.Get(CameraCharacteristics.FlashInfoAvailable) as Java.Lang.Boolean);
                cameraInfo.AvailableResolutions = new();
                try
                {
                    float[] maxFocus = (float[])chars.Get(CameraCharacteristics.LensInfoAvailableFocalLengths);
                    SizeF size = (SizeF)chars.Get(CameraCharacteristics.SensorInfoPhysicalSize);
                    cameraInfo.HorizontalViewAngle = (float)(2 * Math.Atan(size.Width / (maxFocus[0] * 2)));
                    cameraInfo.VerticalViewAngle = (float)(2 * Math.Atan(size.Height / (maxFocus[0] * 2)));
                }
                catch { }
                try
                {
                    StreamConfigurationMap map = (StreamConfigurationMap)chars.Get(CameraCharacteristics.ScalerStreamConfigurationMap);
                    foreach (var s in map.GetOutputSizes(Class.FromType(typeof(ImageReader))))
                        cameraInfo.AvailableResolutions.Add(new(s.Width, s.Height));
                }
                catch
                {
                    if (cameraInfo.Position == CameraPosition.Back)
                        cameraInfo.AvailableResolutions.Add(new(1920, 1080));
                    cameraInfo.AvailableResolutions.Add(new(1280, 720));
                    cameraInfo.AvailableResolutions.Add(new(640, 480));
                    cameraInfo.AvailableResolutions.Add(new(352, 288));
                }
                cameraView.Cameras.Add(cameraInfo);
                }
            if (OperatingSystem.IsAndroidVersionAtLeast(30))
            {
                cameraView.Microphones.Clear();
                foreach (var device in audioManager.Microphones)
                {
                    cameraView.Microphones.Add(new MicrophoneInfo { Name = "Microphone " + device.Type.ToString() + " " + device.Address, DeviceId = device.Id.ToString() });
                }
            }
            //Microphone = Micros.FirstOrDefault();
            executorService = Executors.NewSingleThreadExecutor();

            initiated = true;
            cameraView.RefreshDevices();
        }
    }


    internal async Task<CameraResult> StartRecordingAsync(string file, Microsoft.Maui.Graphics.Size Resolution, OtherRecordingParameters otherRecordingParameters = null)
    {
        var result = CameraResult.Success;
        if (initiated && !recording)
        {
            var withAudio = otherRecordingParameters?.WithAudio ?? true;
            if (await CameraView.RequestPermissions(withAudio, true))
            {
                if (started) StopCamera();
                if (cameraView.Camera != null)
                {
                    try
                    {
                        camChars = cameraManager.GetCameraCharacteristics(cameraView.Camera.DeviceId);

                        StreamConfigurationMap map = (StreamConfigurationMap)camChars.Get(CameraCharacteristics.ScalerStreamConfigurationMap);
                        var outputSizes = map.GetOutputSizes(Class.FromType(typeof(ImageReader)));
                        videoSize = ChooseVideoSize(outputSizes);
                        recording = true;

                        if (File.Exists(file)) File.Delete(file);

                        if (OperatingSystem.IsAndroidVersionAtLeast(31))
                            mediaRecorder = new MediaRecorder(context);
                        else
                            mediaRecorder = new MediaRecorder();
                        audioManager.Mode = Mode.Normal;

                        //HO changed
                        //mediaRecorder.SetAudioSource(AudioSource.Mic);
                        if (withAudio)
                        {
                            audioManager.Mode = Mode.Normal;
                            mediaRecorder.SetAudioSource(AudioSource.Mic);
                        }

                        mediaRecorder.SetVideoSource(VideoSource.Surface);
                        mediaRecorder.SetOutputFormat(OutputFormat.Mpeg4);
                        mediaRecorder.SetOutputFile(file);

                        //TODO: HO verify this
                        //mediaRecorder.SetVideoEncodingBitRate(10000000);

                        //TODO: HO verify this
                        //mediaRecorder.SetVideoFrameRate(30);
                        mediaRecorder.SetVideoFrameRate(otherRecordingParameters?.Fps ?? 30);

                        var maxVideoSize = ChooseMaxVideoSize(map.GetOutputSizes(Class.FromType(typeof(ImageReader))));
                        if (Resolution.Width != 0 && Resolution.Height != 0)
                            maxVideoSize = new((int)Resolution.Width, (int)Resolution.Height);
                        mediaRecorder.SetVideoSize(maxVideoSize.Width, maxVideoSize.Height);

                        //TODO: HO verify this
                        //HO test setCaptureRate also
                        mediaRecorder.SetVideoEncodingBitRate(otherRecordingParameters?.HeightToDesiredBitrateFunc?.Invoke((int)Resolution.Height) ?? 10_000_000);

                        var strEncoder = otherRecordingParameters?.SupportedVideoCodecs?.FirstOrDefault();
                        var encoder = strEncoder switch
                        {
                            "hevc" => VideoEncoder.Hevc,
                            "h264" => VideoEncoder.H264,
                            _ => VideoEncoder.H264
                        };

                        mediaRecorder.SetVideoEncoder(encoder);

                        //HO changed
                        //mediaRecorder.SetAudioEncoder(AudioEncoder.Aac);
                        if (withAudio)
                        {
                            mediaRecorder.SetAudioEncoder(AudioEncoder.Aac);
                        }


                        IWindowManager windowManager = context.GetSystemService(Context.WindowService).JavaCast<IWindowManager>();


                        //HO changed
                        //int rotation = (int)windowManager.DefaultDisplay.Rotation;
                        int rotation = otherRecordingParameters?.RotationRelativeToPortrait ?? (int)windowManager.DefaultDisplay.Rotation;

                        int orientation = cameraView.Camera.Position == CameraPosition.Back ? orientation = ORIENTATIONS.Get(rotation) : orientation = ORIENTATIONSFRONT.Get(rotation);
                        mediaRecorder.SetOrientationHint(orientation);
                        mediaRecorder.Prepare();

                        if (OperatingSystem.IsAndroidVersionAtLeast(28))
                            cameraManager.OpenCamera(cameraView.Camera.DeviceId, executorService, stateListener);
                        else
                            cameraManager.OpenCamera(cameraView.Camera.DeviceId, stateListener, null);
                        started = true;
                    }
                    catch
                    {
                        result = CameraResult.AccessError;
                    }
                }
                else
                    result = CameraResult.NoCameraSelected;
            }
            else
                result = CameraResult.AccessDenied;
        }
        else
            result = CameraResult.NotInitiated;

        return result;
    }

    
    private void StartPreview()
    {
        _logger_LogTrace?.Invoke($"{nameof(StartPreview)}: entered");

        while (textureView.SurfaceTexture == null || !textureView.IsAvailable) Thread.Sleep(100);
        SurfaceTexture texture = textureView.SurfaceTexture;
        texture.SetDefaultBufferSize(videoSize.Width, videoSize.Height);

        previewBuilder = cameraDevice.CreateCaptureRequest(recording ? CameraTemplate.Record : CameraTemplate.Preview);
        if(_setFocusContext.CameraCharacteristics != null)
        {
            if(_setFocusContext.ControlMaxRegionsAf > 0)
            { //HO get a fresh set of OrgAfRegions if they differ between recording and picture taking
                _setFocusContext.OrgAfRegions = previewBuilder.Get(CaptureRequest.ControlAfRegions);
            }
        }

        //HO Old Capture.Android
        //currentCaptureRequest = cameraDevice.CreateCaptureRequest(CameraTemplate.Preview);
        //currentCaptureRequest.Set(CaptureRequest.ControlAfMode, (int)ControlAFMode.ContinuousVideo);
        //currentCaptureRequest.Set(CaptureRequest.ControlAeMode, (int)ControlAEMode.On);
        var surfaces = new List<OutputConfiguration>();
        var surfaces26 = new List<Surface>();
        var previewSurface = new Surface(texture);
        surfaces.Add(new OutputConfiguration(previewSurface));
        surfaces26.Add(previewSurface);
        previewBuilder.AddTarget(previewSurface);
        if (imgReader != null)
        {
            surfaces.Add(new OutputConfiguration(imgReader.Surface));
            surfaces26.Add(imgReader.Surface);
        }
        if (mediaRecorder != null)
        {
            surfaces.Add(new OutputConfiguration(mediaRecorder.Surface));
            surfaces26.Add(mediaRecorder.Surface);
            previewBuilder.AddTarget(mediaRecorder.Surface);
        }
        //HO added;
        if(recording && ((cameraView?.TorchEnabled)  ?? false))
        {
            _logger_LogTrace?.Invoke($"{nameof(StartPreview)}: {nameof(cameraView.TorchEnabled)}: Turning it on");
            try
            {
                previewBuilder.Set(CaptureRequest.ControlAeMode, (int)ControlAEMode.On);
                previewBuilder.Set(CaptureRequest.FlashMode, cameraView.TorchEnabled ? (int)ControlAEMode.OnAutoFlash : (int)ControlAEMode.Off);
            }
            catch (Exception ex)
            { //HO happens sometimes investigate why later
                _logger.LogWarning(ex, "calling UpdateTorch from StartCameraAsync failed");
            }
        }

        sessionCallback = new PreviewCaptureStateCallback(this, _logger, _logger_LogTrace);
        try
        {
            if (OperatingSystem.IsAndroidVersionAtLeast(28))
            {
                SessionConfiguration config = new((int)SessionType.Regular, surfaces, executorService, sessionCallback);
                cameraDevice?.CreateCaptureSession(config);
            }
            else
            {
#pragma warning disable CS0618 // El tipo o el miembro están obsoletos
                cameraDevice.CreateCaptureSession(surfaces26, sessionCallback, null);
#pragma warning restore CS0618 // El tipo o el miembro están obsoletos
            }
        }
        catch (Exception ex)
        { //HO Happens sometimes when going back from preview when user has not saved and discard video is done
            // Should be investigated further
            _logger_LogTrace?.Invoke($"cameraDevice?.CreateCaptureSession FAILED: {ex.Message}");
        }
        _logger_LogTrace?.Invoke("_previewStartedTcs?.TrySetResult()");
        lock(_previewStartedTcsLock)
        {
            _previewStartedTcs?.TrySetResult();
        }
    }
    private void UpdatePreview()
    {
        lock(cameraDeviceLock)
        { //HO gotta cameraDeviceLock otherwise we sometimes get exception in SetZoomFactor cause StopCamera is called in parallell
            if (null == cameraDevice)
                return;

            try
            {
                previewBuilder.Set(CaptureRequest.ControlMode, Java.Lang.Integer.ValueOf((int)ControlMode.Auto));
                //Rect m = (Rect)camChars.Get(CameraCharacteristics.SensorInfoActiveArraySize);
                //videoSize = new Size(m.Width(), m.Height());
                //AdjustAspectRatio(videoSize.Width, videoSize.Height);
                AdjustAspectRatio(videoSize.Width, videoSize.Height);
                SetZoomFactor(cameraView.ZoomFactor);
                //previewSession.SetRepeatingRequest(previewBuilder.Build(), null, null);
                if (recording)
                    mediaRecorder?.Start();
            }
            catch (Exception ex)
            {
                _logger.LogWarning(ex, $"{nameof(UpdatePreview)}: exception");
            }
        }
    }

    object _previewStartedTcsLock = new object();
    TaskCompletionSource _previewStartedTcs = null;
    public TaskCompletionSource _onConfiguredTcs = null;


    Size _cameraViewSizeInPixels = new Size(0, 0);

    internal async Task<CameraResult> StartCameraAsync(Microsoft.Maui.Graphics.Size PhotosResolution)
    {
        var result = CameraResult.Success;
        if (initiated)
        {
            if (await CameraView.RequestPermissions())
            {
                if (started) StopCamera();
                if (cameraView.Camera != null)
                {
                    //HO we only calculate this once it will always be same
                    //HO Reason is CalculateCameraViewSizeInPixels sometimes fail when repeatingly rec - keeep - rec - keep -rec
                    _cameraViewSizeInPixels = (_cameraViewSizeInPixels.Width == 0) ? CalculateCameraViewSizeInPixels() : _cameraViewSizeInPixels;
                    try
                    {
                        camChars = cameraManager.GetCameraCharacteristics(cameraView.Camera.DeviceId);

                        StreamConfigurationMap map = (StreamConfigurationMap)camChars.Get(CameraCharacteristics.ScalerStreamConfigurationMap);

                        if (cameraView.BarCodeDetectionEnabled)
                        { //HO otherwise the quality is to low for barcodescanner sw
                            videoSize = ChooseMaxVideoSize(map.GetOutputSizes(Class.FromType(typeof(ImageReader))));
                        }
                        else
                        {
                            videoSize = ChooseVideoSize(map.GetOutputSizes(Class.FromType(typeof(ImageReader))));
                        }

                        var imgReaderSize = ChooseMaxVideoSize(map.GetOutputSizes(Class.FromType(typeof(ImageReader))));

                        Rect sensorRect = (Rect)camChars.Get(CameraCharacteristics.SensorInfoActiveArraySize);
                        if(_logger.IsEnabled(LogLevel.Trace))
                        {
                            _logger_LogTrace?.Invoke($"{nameof(StartCameraAsync)}: {nameof(sensorRect)}: w: {sensorRect.Width()} h: {sensorRect.Height()}");
                        }
                        if (PhotosResolution.Width != 0 && PhotosResolution.Height != 0)
                        {
                            imgReaderSize = new((int)PhotosResolution.Width, (int)PhotosResolution.Height);
                        }
                        else
                        {   //HO changed to include non bursting sizes, otherwise  Samsung A34 gives images of size maxVideo ( which is less resolution)
                            var jpegSizes = GetAvailableJpegSizes(map);
                            imgReaderSize = jpegSizes.Last();
                        }


                        imgReader = ImageReader.NewInstance(imgReaderSize.Width, imgReaderSize.Height, ImageFormatType.Jpeg, 1);
                        backgroundThread = new HandlerThread("CameraBackground");
                        backgroundThread.Start();
                        backgroundHandler = new Handler(backgroundThread.Looper);
                        imgReader.SetOnImageAvailableListener(photoListener, backgroundHandler);

                        //HO _previewStartedTcs: needed or we sometimes get exception cause StartCameraAsync is called when StopRecording is called 
                        //HO and after StopRecording app directly changes page to PhotoPreviewPage
                        //HO OpenCamera causes an async call to StartPreview via callback which can then not be performed as we have left the CameraPage 
                        //HO This causes exception  
                        //HO Solution we wait for the preview to start before returning from this function
                        lock (_previewStartedTcsLock)
                        {
                            _previewStartedTcs = new();
                            _onConfiguredTcs = new();
                        }

                        if (OperatingSystem.IsAndroidVersionAtLeast(28))
                            cameraManager.OpenCamera(cameraView.Camera.DeviceId, executorService, stateListener);
                        else
                            cameraManager.OpenCamera(cameraView.Camera.DeviceId, stateListener, null);

                        _logger_LogTrace?.Invoke("PRE: await _previewStartedTcs.Task");
                        //HO OnConfigured is called sometime after camera is started this means we have a preview session and can call UpdateTorch
                        await Task.WhenAny(Task.Delay(2000), Task.WhenAll(_onConfiguredTcs?.Task, _previewStartedTcs?.Task));
                        _logger_LogTrace?.Invoke("POST: await _previewStartedTcs.Task");

                        timer.Start();

                        started = true;
                        
                        //HO Added UpdateTorch //so when we reenter camera page it will always light up again
                        UpdateTorch();
                    }
                    catch
                    {
                        result = CameraResult.AccessError;
                    }
                }
                else
                    result = CameraResult.NoCameraSelected;
            }
            else
                result = CameraResult.AccessDenied;
        }
        else
            result = CameraResult.NotInitiated;

        return result;
    }

    private Size[] GetAvailableJpegSizes(StreamConfigurationMap map)
    {
        var jpegSizes = new List<Size>();

        var sizes = map.GetHighResolutionOutputSizes((int)ImageFormatType.Jpeg);

        if (sizes != null)
        {
            jpegSizes.AddRange(sizes);
        }

        sizes = map.GetOutputSizes((int)ImageFormatType.Jpeg);

        if (sizes != null)
        {
            jpegSizes.AddRange(sizes);
        }


        var jpegSizesArray = jpegSizes.OrderBy(x => x.Width * x.Height).ToArray();
        return jpegSizesArray;
    }

    internal Task<CameraResult> StopRecordingAsync()
    {
        recording = false;
        return StartCameraAsync(cameraView.PhotosResolution);
    }

    internal CameraResult StopCamera()
    {
        _logger_LogTrace?.Invoke("StopCamera: before");
        CameraResult result = CameraResult.Success;
        lock(cameraDeviceLock)
        {
            _logger_LogTrace?.Invoke("StopCamera: before: after cameraDeviceLock");
            if (initiated)
            {
                timer.Stop();
                try
                {
                    mediaRecorder?.Stop();
                    mediaRecorder?.Dispose();
                    mediaRecorder = null;
                } catch { }
                try
                {
                    backgroundThread?.QuitSafely();
                    backgroundThread?.Join();
                    backgroundThread = null;
                    backgroundHandler = null;
                    imgReader?.Dispose();
                    imgReader = null;
                }
                catch { }
                try
                {
                    previewSession?.StopRepeating();
                    previewSession?.AbortCaptures();
                    previewSession?.Dispose();
                    previewSession = null;
                } catch { }
                try
                {
                    cameraDevice?.Close();
                    cameraDevice?.Dispose();
            } catch { }
                previewSession = null;
                cameraDevice = null;
                previewBuilder = null;
                mediaRecorder = null;
                started = false;
                recording = false;
            }
            else
                result = CameraResult.NotInitiated;
            _logger_LogTrace?.Invoke($"StopCamera: after");

            return result;
        }
    }
    internal void DisposeControl()
    {
        try
        {
            if (started) StopCamera();
            executorService?.Shutdown();
            executorService?.Dispose();
            RemoveAllViews();
            textureView?.Dispose();
            timer?.Dispose();
            Dispose();
        }
        catch { }
    }
    private void ProccessQR()
    {
        Task.Run(() =>
        {
            Bitmap bitmap = TakeSnap();
            if (bitmap != null)
            {
                System.Diagnostics.Debug.WriteLine($"Processing QR ({bitmap.Width}x{bitmap.Height}) " + DateTime.Now.ToString("mm:ss:fff"));
                cameraView.DecodeBarcode(bitmap);
                bitmap.Dispose();
                System.Diagnostics.Debug.WriteLine("QR Processed " + DateTime.Now.ToString("mm:ss:fff"));
            }
            lock (cameraView.currentThreadsLocker) cameraView.currentThreads--;
        });
    }
    private void RefreshSnapShot()
    {
        cameraView.RefreshSnapshot(GetSnapShot(cameraView.AutoSnapShotFormat, true));
    }

    private void Timer_Elapsed(object sender, System.Timers.ElapsedEventArgs e)
    {
        if (!snapping && cameraView != null && cameraView.AutoSnapShotSeconds > 0 && (DateTime.Now - cameraView.lastSnapshot).TotalSeconds >= cameraView.AutoSnapShotSeconds)
        {
            Task.Run(() => RefreshSnapShot());
        }
        else if (cameraView.BarCodeDetectionEnabled)
        {
            frames++;
            if (frames >= cameraView.BarCodeDetectionFrameRate)
            {
                bool processQR = false;
                lock (cameraView.currentThreadsLocker)
                {
                    if (cameraView.currentThreads < cameraView.BarCodeDetectionMaxThreads)
                    {
                        cameraView.currentThreads++;
                        processQR = true;
                    }
                }
                if (processQR)
                {
                    ProccessQR();
                    frames = 0;
                }
            }
        }

    }


    private Bitmap TakeSnap()
    {
        Bitmap bitmap = null;
        try
        {
            MainThread.InvokeOnMainThreadAsync(() => {
                //HO??? why did hjam40 make this first call next call replaces the bitmap anyway 
                //bitmap = textureView.GetBitmap(null);
                //bitmap?.Dispose();
                //_logger.LogInformation($"0. bitmap.Width:{bitmap?.Width} bitmap.Height: {bitmap?.Height}");

                bitmap = textureView.Bitmap;

                //_logger.LogInformation($"1. bitmap.Width:{bitmap?.Width} bitmap.Height: {bitmap?.Height}");
            }).Wait();
            _logger.LogInformation($"2. bitmap.Width:{bitmap.Width} bitmap.Height: {bitmap.Height}");
            if (bitmap != null)
            {
                //_logger.LogInformation($"3. bitmap.Width:{bitmap.Width} bitmap.Height: {bitmap.Height}");
                int oriWidth = bitmap.Width;
                int oriHeight = bitmap.Height;

                //HO this call crops the bitmap according to what is visible in textureView 
                //if AspectFitPreview this is smaller than android view
                //if not AspectFitPreview this is bigger than android view
                {
                    var prevBitmap = bitmap;
                    bitmap = Bitmap.CreateBitmap(bitmap, 0, 0, bitmap.Width, bitmap.Height, textureView.GetTransform(null), false);
                    prevBitmap.Dispose();
                }
                //_logger.LogInformation($"4. bitmap.Width:{bitmap.Width} bitmap.Height: {bitmap.Height}");

                //HO this orig code is wrong cause doesnt adjust the offset!
                //bitmap = Bitmap.CreateBitmap(bitmap, 0, 0, Width, Height);
                //_logger.LogInformation($"5. bitmap.Width:{bitmap.Width} bitmap.Height: {bitmap.Height}");

                if (textureView.ScaleX == -1)
                {
                    Matrix matrix = new();
                    matrix.PreScale(-1, 1);
                    var prevBitmap = bitmap;
                    bitmap = Bitmap.CreateBitmap(bitmap, 0, 0, bitmap.Width, bitmap.Height, matrix, false);
                    prevBitmap.Dispose();
                }
            }
        }
        catch { }
        return bitmap;
    }

    private static Microsoft.Maui.Graphics.Rect CalculateScalerRect(Rect sensorRect, Single zoomFactor)
    {
        Rect m = sensorRect;

        var ratio = 1 / zoomFactor;
        var sensorRectWidth = sensorRect.Right - sensorRect.Left;
        var sensorRectHeight = sensorRect.Bottom - sensorRect.Top;

        var w = (int)(sensorRectWidth * ratio);
        var newLeft = (sensorRectWidth - w) / 2;
        var h = (int)(sensorRectHeight * ratio);
        var newTop = (sensorRectHeight - h) / 2;
        return new Microsoft.Maui.Graphics.Rect(newLeft, newTop, w + newLeft, h + newTop);
    }

    string CreateLogStringForRect(Rect rect)
    {
        return $"|L:{rect.Left} T:{rect.Top} R:{rect.Right} B:{rect.Bottom} W:{rect.Width()} H:{rect.Height()}|";
    }


    internal async Task<System.IO.Stream> TakePhotoAsync(ImageFormat imageFormat, int? rotation)
    {
        MemoryStream stream = null;
        if (started && !recording)
        {
            CaptureRequest.Builder singleRequest = cameraDevice.CreateCaptureRequest(CameraTemplate.StillCapture);
            //singleRequest.AddTarget(imgReader.Surface);

            captureDone = false;
            capturePhoto = null;
            if (cameraView.Camera.HasFlashUnit)
            {
                switch (cameraView.FlashMode)
                {
                    case FlashMode.Auto:
                        singleRequest.Set(CaptureRequest.FlashMode, (int)ControlAEMode.OnAutoFlash);
                        break;
                    case FlashMode.Enabled:
                        singleRequest.Set(CaptureRequest.FlashMode, (int)ControlAEMode.On);
                        break;
                    case FlashMode.Disabled:
                        singleRequest.Set(CaptureRequest.FlashMode, (int)ControlAEMode.Off);
                        break;
                }
            }

            //HO changed
            if(!rotation.HasValue )
            {
                IWindowManager windowManager = context.GetSystemService(Context.WindowService).JavaCast<IWindowManager>();
                var displayRotation = windowManager.DefaultDisplay.Rotation;
                rotation = GetJpegOrientation(displayRotation);
            }
            else
            {
                var displayRotation = rotation switch {
                    90 => SurfaceOrientation.Rotation90,
                    180 => SurfaceOrientation.Rotation180,
                    270 => SurfaceOrientation.Rotation270,
                    _ => SurfaceOrientation.Rotation0
                }; 
                rotation = GetJpegOrientation(displayRotation);
            }

            singleRequest.Set(CaptureRequest.JpegOrientation, rotation);

            var destZoom = Math.Clamp(cameraView.ZoomFactor, 1, cameraView.Camera.MaxZoomFactor);
            if (OperatingSystem.IsAndroidVersionAtLeast(_useControlZoomRatio_ApiLevel))
            {
                singleRequest.Set(CaptureRequest.ControlZoomRatio, destZoom);
            }
            else
            {
                Rect sensorRect = (Rect)camChars.Get(CameraCharacteristics.SensorInfoActiveArraySize);
                Rect zoomedSensorArea = CalculateScalerRect(sensorRect, destZoom);
                singleRequest.Set(CaptureRequest.ScalerCropRegion, zoomedSensorArea);
            }

            singleRequest.AddTarget(imgReader.Surface);
            try
            {
                var captureRequest = singleRequest.Build();
                previewSession.Capture(captureRequest, null, null);
                while (!captureDone) await Task.Delay(50);
                if (capturePhoto != null)
                {

                    _logger_LogTrace?.Invoke($"{nameof(TakePhotoAsync)}: 20: TextureView: {textureView.ToString()} ScaleX:{textureView.ScaleX} ScaleY: {textureView.ScaleY}");
                    if (textureView.ScaleX == -1 || imageFormat != ImageFormat.JPEG)
                    {
                        Bitmap bitmap = BitmapFactory.DecodeByteArray(capturePhoto, 0, capturePhoto.Length);
                        if (bitmap != null)
                        {
                            if (textureView.ScaleX == -1)
                            {
                                Matrix matrix = new();
                                matrix.PreRotate(rotation.Value);
                                matrix.PostScale(-1, 1);
                                var prevBitmap = bitmap;
                                bitmap = Bitmap.CreateBitmap(bitmap, 0, 0, bitmap.Width, bitmap.Height, matrix, false);
                                prevBitmap.Dispose();
                            }
                            var iformat = imageFormat switch
                            {
                                ImageFormat.JPEG => Bitmap.CompressFormat.Jpeg,
                                _ => Bitmap.CompressFormat.Png
                            };
                            stream = new();
                            bitmap.Compress(iformat, 100, stream);
                            stream.Position = 0;
                        }
                    }
                    else
                    {
                        stream = new();
                        stream.Write(capturePhoto);
                        stream.Position = 0;
                    }
                }
            }
            catch(Java.Lang.Exception ex)
            {
                System.Diagnostics.Debug.WriteLine(ex.StackTrace);
            }
        }
        return stream;
    }
    internal ImageSource GetSnapShot(ImageFormat imageFormat, bool auto = false)
    {
        ImageSource result = null;

        if (started && !snapping)
        {
            snapping = true;
            Bitmap bitmap = TakeSnap();

            if (bitmap != null)
            {
                var iformat = imageFormat switch
                {
                    ImageFormat.JPEG => Bitmap.CompressFormat.Jpeg,
                    _ => Bitmap.CompressFormat.Png
                };
                MemoryStream stream = new();
                bitmap.Compress(iformat, 100, stream);
                stream.Position = 0;
                if (auto)
                {
                    if (cameraView.AutoSnapShotAsImageSource)
                        result = ImageSource.FromStream(() => stream);
                    cameraView.SnapShotStream?.Dispose();
                    cameraView.SnapShotStream = stream;
                }
                else
                    result = ImageSource.FromStream(() => stream);
                bitmap.Dispose();
            }
            snapping = false;
        }
        return result;
    }

    internal bool SaveSnapShot(ImageFormat imageFormat, string SnapFilePath, int? rotation)
    {
        bool result = true;

        if (started && !snapping)
        {
            snapping = true;
            Bitmap bitmap = TakeSnap();
            if (bitmap != null)
            {
                if (!rotation.HasValue)
                {
                    IWindowManager windowManager = context.GetSystemService(Context.WindowService).JavaCast<IWindowManager>();
                    rotation = (int)windowManager.DefaultDisplay.Rotation;
                }

                Matrix matrix = null;
                if (rotation.Value != 0)
                {
                    matrix ??= new();
                    //dont now why but using 360 - gives the correct value together with the recording
                    matrix.PostRotate(360 - rotation.Value, bitmap.Width / 2, bitmap.Height / 2);
                }


                if(matrix != null)
                {
                    //HO Recommended default is to set filter to 'true' as the cost of bilinear filtering is typically minimal and the improved image quality is significant.
                    var prevBimap = bitmap;
                    bitmap = Bitmap.CreateBitmap(bitmap, 0, 0, bitmap.Width, bitmap.Height, matrix, true);
                    prevBimap.Dispose();
                }

                if (File.Exists(SnapFilePath)) File.Delete(SnapFilePath);
                var iformat = imageFormat switch
                {
                    ImageFormat.JPEG => Bitmap.CompressFormat.Jpeg,
                    _ => Bitmap.CompressFormat.Png
                };
                using FileStream stream = new(SnapFilePath, FileMode.OpenOrCreate);
                bitmap.Compress(iformat, 80, stream);
                stream.Close();
            }
            snapping = false;
        }
        else
            result = false;

        return result;
    }
    public void UpdateMirroredImage()
    {
        if (cameraView != null && textureView != null)
        {
            if (cameraView.MirroredImage) 
                textureView.ScaleX = -1;
            else
                textureView.ScaleX = 1;
        }
    }
    internal void UpdateTorch()
    {
        if (cameraView?.Camera?.HasFlashUnit ?? false)
        {
            _logger_LogTrace?.Invoke($"{nameof(UpdateTorch)}: {nameof(cameraView.TorchEnabled)}: Turning it {cameraView.TorchEnabled}");
            if (started)
            {
                _logger_LogTrace?.Invoke($"{nameof(UpdateTorch)}: {nameof(previewBuilder)}: {(previewBuilder != null)}  {nameof(previewSession)}: {(previewSession != null)} ");
                previewBuilder.Set(CaptureRequest.ControlAeMode, (int)ControlAEMode.On);
                previewBuilder.Set(CaptureRequest.FlashMode, cameraView.TorchEnabled ? (int)ControlAEMode.OnAutoFlash : (int)ControlAEMode.Off);
                previewSession.SetRepeatingRequest(previewBuilder.Build(), null, null);
            }
            else if (initiated)
                cameraManager.SetTorchMode(cameraView.Camera.DeviceId, cameraView.TorchEnabled);
        }
    }
    internal void UpdateFlashMode()
    {
        if (previewSession != null && previewBuilder != null && cameraView.Camera != null && cameraView != null)
        {
            try
            {
                if (cameraView.Camera.HasFlashUnit)
                {
                    switch (cameraView.FlashMode)
                    {
                        case FlashMode.Auto:
                            previewBuilder.Set(CaptureRequest.ControlAeMode, (int)ControlAEMode.OnAutoFlash);
                            previewSession.SetRepeatingRequest(previewBuilder.Build(), null, null);
                            break;
                        case FlashMode.Enabled:
                            previewBuilder.Set(CaptureRequest.ControlAeMode, (int)ControlAEMode.On);
                            previewSession.SetRepeatingRequest(previewBuilder.Build(), null, null);
                            break;
                        case FlashMode.Disabled:
                            previewBuilder.Set(CaptureRequest.ControlAeMode, (int)ControlAEMode.Off);
                            previewSession.SetRepeatingRequest(previewBuilder.Build(), null, null);
                            break;
                    }
                }
            }
            catch (System.Exception)
            {
            }
        }
    }
    internal void SetZoomFactor(float zoom)
    {
        lock(cameraDeviceLock)
        { //HO lock cause sometimes get Android.Hardware.Camera2.CameraAccessException: 'CAMERA_DISCONNECTED (2): checkPidStatus:2384: The camera device has been disconnected'
            _logger_LogTrace?.Invoke("SetZoomFactor BEGIN");
            if (previewSession != null && previewBuilder != null && cameraView?.Camera != null)
            {
                var destZoom = Math.Clamp(cameraView.ZoomFactor, 1, cameraView.Camera.MaxZoomFactor);
                if (OperatingSystem.IsAndroidVersionAtLeast(_useControlZoomRatio_ApiLevel))
                {
                    previewBuilder.Set(CaptureRequest.ControlZoomRatio, destZoom);
                }
                else
                {
                    Rect sensorRect = (Rect)camChars.Get(CameraCharacteristics.SensorInfoActiveArraySize);
                    Rect zoomedSensorArea = CalculateScalerRect(sensorRect, destZoom);
                    previewBuilder.Set(CaptureRequest.ScalerCropRegion, zoomedSensorArea);
                }

                previewSession.SetRepeatingRequest(previewBuilder.Build(), null, null);
                //HO focus must be updated as it is a point relative to the preview not the sensor
                SetFocus(_focusRect);
            }
            _logger_LogTrace?.Invoke("SetZoomFactor END");
        }
    }
    internal void ForceAutoFocus()
    {
        if (previewSession != null && previewBuilder != null && cameraView.Camera != null)
        {
            previewBuilder.Set(CaptureRequest.ControlAfMode, Java.Lang.Integer.ValueOf((int)ControlAFMode.Off));
            previewBuilder.Set(CaptureRequest.ControlAfTrigger, Java.Lang.Integer.ValueOf((int)ControlAFTrigger.Cancel));
            previewSession.SetRepeatingRequest(previewBuilder.Build(), null, null);
            previewBuilder.Set(CaptureRequest.ControlAfMode, Java.Lang.Integer.ValueOf((int)ControlAFMode.Auto));
            previewBuilder.Set(CaptureRequest.ControlAfTrigger, Java.Lang.Integer.ValueOf((int)ControlAFTrigger.Start));
            previewSession.SetRepeatingRequest(previewBuilder.Build(), null, null);

        }
    }
    private static Size ChooseMaxVideoSize(Size[] choices)
    {
        Size result = choices[0];
        int diference = 0;

        foreach (Size size in choices)
        {
            if (size.Width == size.Height * 4 / 3 && size.Width * size.Height > diference)
            {
                result = size;
                diference = size.Width * size.Height;
            }
        }

        return result;
    }

    Size CalculateCameraViewSizeInPixels()
    {
        var cameraViewWidthInPixels = (int)Math.Round(cameraView.Width * DeviceDisplay.Current.MainDisplayInfo.Density);
        var cameraViewHeightInPixels = (int)Math.Round(cameraView.Height * DeviceDisplay.Current.MainDisplayInfo.Density);

        if (cameraView.Width > cameraView.Height)
        {
            cameraViewWidthInPixels = (int)Math.Round(cameraView.Height * DeviceDisplay.Current.MainDisplayInfo.Density);
            cameraViewHeightInPixels = (int)Math.Round(cameraView.Width * DeviceDisplay.Current.MainDisplayInfo.Density);
        }

        return new Size(cameraViewWidthInPixels, cameraViewHeightInPixels);
    }


    private Size ChooseVideoSize(Size[] choices)
    {
        bool swapped = IsDimensionSwapped();

        //from smaller to larger
        choices = choices.OrderBy(x => x.Width * x.Height).ToArray();

        Size result = choices[0];



        var cameraViewSizeInPixels_Width = _cameraViewSizeInPixels.Width;
        var cameraViewSizeInPixels_Height = _cameraViewSizeInPixels.Height;

        if (swapped)
        {
            cameraViewSizeInPixels_Width = _cameraViewSizeInPixels.Height;
            cameraViewSizeInPixels_Height = _cameraViewSizeInPixels.Width;
        }

        bool hasFoundBestSensorResolution = false;
        foreach (Size size in choices)
        {
            bool isSensorResolution = size.Width == Math.Round((float)size.Height * 4 / 3);
            if (isSensorResolution && !hasFoundBestSensorResolution)
            {
                //HO here we find a resolution that matches Android view size (in pixels)
                //HO Old code looked at the Width/Heighr of the android view which may be 0 when entering here first time
                //Android view Width Height is given in pixels so convert (maui) cameraView dimensions to pixels
                if ((size.Width >= cameraViewSizeInPixels_Width) && (size.Height >= cameraViewSizeInPixels_Height))
                {
                    hasFoundBestSensorResolution = true;
                }
                result = size;
            }
        }

        _logger_LogTrace?.Invoke($"{nameof(ChooseVideoSize)}: selected: w:{result?.Width} h:{result?.Height}");
        return result;
    }

    private void AdjustAspectRatio(int videoWidth, int videoHeight)
    {
        Matrix txform = new();
        /*
        float scaleX = (float)videoWidth / Width;
        float scaleY = (float)videoHeight / Height;
        bool swapped = IsDimensionSwapped();
        if (swapped)
        {
            scaleX = (float)videoHeight / Width;
            scaleY = (float)videoWidth / Height;
        }
        if (scaleX <= scaleY)
        {
            scaleY /= scaleX;
            scaleX = 1;
        }
        else
        {
            scaleX /= scaleY;
            scaleY = 1;
        }
        */
        bool isSwapped = IsDimensionSwapped();



        RectF viewRect = new(0, 0, (float)_cameraViewSizeInPixels.Width, (float)_cameraViewSizeInPixels.Height);
        float centerX = viewRect.CenterX();
        float centerY = viewRect.CenterY();

        RectF bufferRect = new(0, 0, videoHeight, videoWidth);
        bufferRect.Offset(centerX - bufferRect.CenterX(), centerY - bufferRect.CenterY());
        txform.SetRectToRect(viewRect, bufferRect, Matrix.ScaleToFit.Fill);

        //HO changed
        /*
        */
        float scale;

        if (cameraView.AspectFitPreview)
        {
            scale = Math.Min(
                    (float)_cameraViewSizeInPixels.Height / (isSwapped ? videoWidth : videoHeight),
                    (float)_cameraViewSizeInPixels.Width / (isSwapped ? videoHeight : videoWidth));
        }
        else
        {
            scale = Math.Max(
                    (float)_cameraViewSizeInPixels.Height / (isSwapped ? videoWidth :  videoHeight),
                    (float)_cameraViewSizeInPixels.Width / (isSwapped ? videoHeight : videoWidth));
        }
        txform.PostScale(scale, scale, centerX, centerY);

        //txform.PostScale(scaleX, scaleY, centerX, centerY);
        /*HO this is not needed we are always showing in portrait mode
        IWindowManager windowManager = context.GetSystemService(Context.WindowService).JavaCast<IWindowManager>();
        var rotation = windowManager.DefaultDisplay.Rotation;
        if (SurfaceOrientation.Rotation90 == rotation || SurfaceOrientation.Rotation270 == rotation)
        {
            txform.PostRotate(90 * ((int)rotation - 2), centerX, centerY);
        }
        else if (SurfaceOrientation.Rotation180 == rotation)
        {
            txform.PostRotate(180, centerX, centerY);
        }
        */
        textureView.SetTransform(txform);
    }

    protected override async void OnConfigurationChanged(Configuration newConfig)
    {
        base.OnConfigurationChanged(newConfig);
        if (started && !recording)
            await StartCameraAsync(cameraView.PhotosResolution);
    }

    private bool IsDimensionSwapped()
    {
        IWindowManager windowManager = context.GetSystemService(Context.WindowService).JavaCast<IWindowManager>();
        var displayRotation = windowManager.DefaultDisplay.Rotation;
        var chars = cameraManager.GetCameraCharacteristics(cameraView.Camera.DeviceId);
        int sensorOrientation = (int)(chars.Get(CameraCharacteristics.SensorOrientation) as Java.Lang.Integer);
        bool swappedDimensions = false;
        switch(displayRotation)
        {
            case SurfaceOrientation.Rotation0:
            case SurfaceOrientation.Rotation180:
                if (sensorOrientation == 90 || sensorOrientation == 270)
                {
                    swappedDimensions = true;
                }
                break;
            case SurfaceOrientation.Rotation90:
            case SurfaceOrientation.Rotation270:
                if (sensorOrientation == 0 || sensorOrientation == 180)
                {
                    swappedDimensions = true;
                }
                break;
        }
        return swappedDimensions;
    }
    private int GetJpegOrientation(SurfaceOrientation displayRotation)
    {
        var chars = cameraManager.GetCameraCharacteristics(cameraView.Camera.DeviceId);
        int sensorOrientation = (int)(chars.Get(CameraCharacteristics.SensorOrientation) as Java.Lang.Integer);
        int deviceOrientation = displayRotation switch
        {
            SurfaceOrientation.Rotation90 => 0,
            SurfaceOrientation.Rotation180 => 270,
            SurfaceOrientation.Rotation270 => 180,
            _ => 90
        };
        // Round device orientation to a multiple of 90
        //deviceOrientation = (deviceOrientation + 45) / 90 * 90;

        // Reverse device orientation for front-facing cameras
        //if (cameraView.Camera.Position == CameraPosition.Front) deviceOrientation = -deviceOrientation;

        // Calculate desired JPEG orientation relative to camera orientation to make
        // the image upright relative to the device orientation
        int jpegOrientation = (sensorOrientation + deviceOrientation + 270) % 360;

        return jpegOrientation;
    }
    private class MyCameraStateCallback : CameraDevice.StateCallback
    {
        private readonly MauiCameraView cameraView;
        private readonly ILogger _logger;
        Action<string> _logger_LogTrace = null;

        public MyCameraStateCallback(MauiCameraView camView, ILogger logger, Action<string> logger_LogTrace)
        {
            cameraView = camView;
            _logger = logger;
            _logger_LogTrace = logger_LogTrace;
        }
        public override void OnOpened(CameraDevice camera)
        {
            _logger_LogTrace?.Invoke($"{nameof(MyCameraStateCallback)}: {nameof(OnOpened)}: entered");
            if (camera != null)
            {
                cameraView.cameraDevice = camera;
                cameraView.StartPreview();
            }
        }

        public override void OnDisconnected(CameraDevice camera)
        {
            camera.Close();
            cameraView.cameraDevice = null;
        }

        public override void OnError(CameraDevice camera, CameraError error)
        {
            camera?.Close();
            cameraView.cameraDevice = null;
        }
    }

    private class PreviewCaptureStateCallback : CameraCaptureSession.StateCallback
    {
        private readonly MauiCameraView cameraView;
        ILogger _logger;
        Action<string> _logger_LogTrace = null;

        public PreviewCaptureStateCallback(MauiCameraView camView, ILogger logger, Action<string> logger_LogTrace)
        {
            cameraView = camView;
            _logger = logger;
            _logger_LogTrace = logger_LogTrace;
        }
        public override void OnConfigured(CameraCaptureSession session)
        {
            lock(cameraView.cameraDeviceLock)
            { 
                _logger_LogTrace?.Invoke($"OnConfigured: begin");
                cameraView.previewSession = session;
                cameraView.UpdatePreview();
                _logger_LogTrace?.Invoke("cameraView._onConfiguredTcs.TrySetResult(); PRE");
                cameraView._onConfiguredTcs?.TrySetResult();
                _logger_LogTrace?.Invoke("cameraView._onConfiguredTcs.TrySetResult(); POST");
                _logger_LogTrace?.Invoke($"OnConfigured: end");
            }
        }
        public override void OnConfigureFailed(CameraCaptureSession session)
        {
        }
    }
    class ImageAvailableListener : Java.Lang.Object, ImageReader.IOnImageAvailableListener
    {
        private readonly MauiCameraView cameraView;

        public ImageAvailableListener(MauiCameraView camView)
        {
            cameraView = camView;
        }
        public void OnImageAvailable(ImageReader reader)
        {
            try
            {
                var image = reader?.AcquireNextImage();
                if (image == null)
                    return;

                var buffer = image.GetPlanes()?[0].Buffer;
                if (buffer == null)
                    return;

                var imageData = new byte[buffer.Capacity()];
                buffer.Get(imageData);
                cameraView.capturePhoto = imageData;
                buffer.Clear();
                image.Close();
            }
            catch
            {
            }
            cameraView.captureDone = true;
        }
    }


    public class SetFocusContext
    {
        public CameraCharacteristics CameraCharacteristics { get; set; }
        public Rect ActiveArraySize { get; set; }
        public int ControlMaxRegionsAf { get; set; }
        public Java.Lang.Object OrgAfRegions { get; set; }
    }

    SetFocusContext _setFocusContext = new SetFocusContext();

#if !TAP_TO_FOCUS_CONTINIOUS
    public bool ManualFocusEngaged { get; set; }

    class ManualFocusEngaged_CaptureCallback : CameraCaptureSession.CaptureCallback
    {
        readonly MauiCameraView _mauiCameraView;
        readonly ILogger _logger;
        readonly SetFocusContext _setFocusContext;

        public ManualFocusEngaged_CaptureCallback(MauiCameraView mauiCameraView, ILogger logger, SetFocusContext setFocusContext)
        {
            _mauiCameraView = mauiCameraView;
            _logger = logger;
            _setFocusContext = setFocusContext;
        }

        public override void OnCaptureCompleted(CameraCaptureSession session, CaptureRequest request, TotalCaptureResult result)
        {
            try
            {
                base.OnCaptureCompleted(session, request, result);
                _mauiCameraView.ManualFocusEngaged = false;

                var requestTag = (string)request.Tag;
                if (requestTag == "FOCUS_TAG")
                {
                    //In some devices(Sony xperia G8142 etc..),CaptureRequest.CONTROL_AF_TRIGGER can not be set to null,or the camera throws an error. but how to solve this, I have no idea..
                    //I found that setting CONTROL_AF_TRIGGER to CONTROL_AF_TRIGGER_IDLE instead of null works on Xperia devices.
                    _mauiCameraView.previewBuilder.Set(CaptureRequest.ControlAfTrigger, (int)ControlAFTrigger.Idle);

                    // (int)ControlAFTrigger.Idle);// bControlAFMode.T CaptureRequest.CONTROL_AF_TRIGGER, null);
                    _mauiCameraView.previewSession.SetRepeatingRequest(_mauiCameraView.previewBuilder.Build(), null, null);
                }
            }
            catch (Exception ex)
            { 
                _logger.LogWarning($"{nameof(OnCaptureCompleted)}: failed: {ex.Message}");
            }

        }
        public override void OnCaptureFailed(CameraCaptureSession session, CaptureRequest request, CaptureFailure failure)
        {
            try
            {
                base.OnCaptureFailed(session, request, failure);
                var failureReason = (string)failure.Reason.ToString();
                _logger.LogWarning($"{nameof(OnCaptureFailed)}: {failureReason}");
                _mauiCameraView.ManualFocusEngaged = false;
            }
            catch (Exception ex)
            {
                _logger.LogWarning($"{nameof(OnCaptureFailed)}: failed: {ex.Message}");
            }
        }
    }
#endif

    Microsoft.Maui.Graphics.Rect _focusRect = Microsoft.Maui.Graphics.Rect.Zero;



    public bool SetFocus(Microsoft.Maui.Graphics.Rect rect)// (Microsoft.Maui.Graphics.PointF pointRelativeToVisibleArea)
    {


        // Get characteristics
        if (_setFocusContext.CameraCharacteristics == null)
        {
            _setFocusContext.CameraCharacteristics = camChars;
            _setFocusContext.ActiveArraySize = (Rect)_setFocusContext.CameraCharacteristics.Get(CameraCharacteristics.SensorInfoActiveArraySize);
            _setFocusContext.ControlMaxRegionsAf = (int)_setFocusContext.CameraCharacteristics.Get(CameraCharacteristics.ControlMaxRegionsAf);
            if(_setFocusContext.ControlMaxRegionsAf > 0)
            {
                _setFocusContext.OrgAfRegions = previewBuilder.Get(CaptureRequest.ControlAfRegions);
            }
        }

        if (_setFocusContext.ControlMaxRegionsAf == 0)
        {
            //no af regions allowed return
            _focusRect = Microsoft.Maui.Graphics.Rect.Zero;
            return false;
        }

        _focusRect = rect;
        if (rect == Microsoft.Maui.Graphics.Rect.Zero)
        { //revert to autofocus
            //HO must set it to the original regions null does not work
            previewBuilder.Set(CaptureRequest.ControlAfRegions, _setFocusContext.OrgAfRegions);
            //previewBuilder.Set(CaptureRequest.ControlAfMode, (int)ControlAFMode.ContinuousPicture);// (int)ControlAFTrigger.Idle);// bControlAFMode.T CaptureRequest.CONTROL_AF_TRIGGER, null);
            previewSession.SetRepeatingRequest(previewBuilder.Build(), null, null);
            return true;
        }




        //HO in this code we assume:
        //   - cameraView.Height always matches sensor.Width
        //   - sensor is rotated 90 degrees relative to phone up
        //   - what we se is the middle of the sensor

        var activeArraySize = _setFocusContext.ActiveArraySize;

        var fakeCameraViewWidthInCameraViewUnits = (cameraView.Height / activeArraySize.Width()) * activeArraySize.Height();
        var fakeXOffsetInCameraViewUnits = (fakeCameraViewWidthInCameraViewUnits - cameraView.Width) / 2;

        var fakeCameraViewX = (rect.Center.X + fakeXOffsetInCameraViewUnits);
        var fakeCameraViewY = (rect.Center.Y);

        //calculate focusrect if somebody wants it


        var sensorTouchAreaExtent = (activeArraySize.Width() * (rect.Height / cameraView.Height));
        var usedSensorArray = new Rect(0, 0, activeArraySize.Width(), activeArraySize.Height());

        //handle zoom
        if (cameraView.ZoomFactor != 1f)
        {
            usedSensorArray = CalculateScalerRect(usedSensorArray, cameraView.ZoomFactor);
            sensorTouchAreaExtent = sensorTouchAreaExtent / cameraView.ZoomFactor;
        }
        //rotate
        var sensorX = (usedSensorArray.Left + (usedSensorArray.Width() * (fakeCameraViewY / cameraView.Height)));
        var sensorY = usedSensorArray.Top + (usedSensorArray.Height() * (1 - (fakeCameraViewX / fakeCameraViewWidthInCameraViewUnits)));


        // Create MeteringRectangle
        var sensorRect = new Microsoft.Maui.Graphics.Rect((sensorX - (sensorTouchAreaExtent / 2)) ,
             (sensorY - (sensorTouchAreaExtent / 2)),
             sensorTouchAreaExtent,
             sensorTouchAreaExtent);


        _logger_LogTrace($"Converting from view coordinates too sensor coordinates");
        _logger_LogTrace($"CameraView Size: {new Microsoft.Maui.Graphics.Size(this.Width, this.Height)} Sensor size: {new Microsoft.Maui.Graphics.Size(activeArraySize.Width(), activeArraySize.Height())}  ");
        _logger_LogTrace($"CameraView Focus Rect: {rect} Sensor Focus Rect: {sensorRect}  ");


        var focusAreaTouch = new MeteringRectangle(
             (int)sensorRect.Left,
             (int)sensorRect.Top,
             (int)sensorRect.Width,
             (int)sensorRect.Height,
             MeteringRectangle.MeteringWeightMax
        );

#if !TAP_TO_FOCUS_CONTINIOUS
        //first stop the existing repeating request
        previewSession.StopRepeating();

        //cancel any existing AF trigger (repeated touches, etc.)
        previewBuilder.Set(CaptureRequest.ControlAfTrigger, (int)ControlAFTrigger.Cancel);// CameraMetadata.Cance //CONTROL_AF_TRIGGER_CANCEL);
        previewBuilder.Set(CaptureRequest.ControlAfMode, (int)ControlAFMode.Off);//CONTROL_AF_MODE_OFF);
        var manualFocusEngagedCaptureCallback = new ManualFocusEngaged_CaptureCallback(this, _logger, _setFocusContext);
        previewSession.Capture(previewBuilder.Build(), manualFocusEngagedCaptureCallback, null);
#endif

#if TAP_TO_FOCUS_CONTINIOUS
        previewBuilder.Set(CaptureRequest.ControlAfRegions, new MeteringRectangle[] { focusAreaTouch });
        previewSession.SetRepeatingRequest(previewBuilder.Build(), null, null);
#else
        //Now add a new AF trigger with focus region
        previewBuilder.Set(CaptureRequest.ControlMode, (int)ControlMode.Auto);  //.CONTROL_MODE, CameraMetadata.CONTROL_MODE_AUTO);
        previewBuilder.Set(CaptureRequest.ControlAfMode, (int)ControlAFMode.Auto); //CONTROL_AF_MODE, CaptureRequest.CONTROL_AF_MODE_AUTO);
        previewBuilder.Set(CaptureRequest.ControlAfTrigger, (int)ControlAFTrigger.Start); //CONTROL_AF_TRIGGER, CameraMetadata.CONTROL_AF_TRIGGER_START);
        previewBuilder.SetTag("FOCUS_TAG"); //we'll capture this later for resuming the preview
        previewBuilder.Set(CaptureRequest.ControlAfRegions, new MeteringRectangle[] { focusAreaTouch });
        //then we ask for a single request (not repeating!)
        previewSession.Capture(previewBuilder.Build(), manualFocusEngagedCaptureCallback, null);
        ManualFocusEngaged = true;
#endif
        return true;
    }
}


