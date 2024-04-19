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

    public MauiCameraView(Context context, CameraView cameraView) : base(context)
    {
        _logger = MauiApplication.Current.Services.GetService<ILogger<MauiCameraView>>();
        this.context = context;
        this.cameraView = cameraView;

        textureView = new(context);
        timer = new(33.3);
        timer.Elapsed += Timer_Elapsed;
        stateListener = new MyCameraStateCallback(this, _logger);
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


    internal async Task<CameraResult> StartRecordingAsync(string file, Microsoft.Maui.Graphics.Size Resolution, int? fps = null, Func<int, int> bitrate = null, bool withAudio = true, int? rotation = null)
    {
        var result = CameraResult.Success;
        if (initiated && !recording)
        {
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
                            mediaRecorder.SetAudioSource(AudioSource.Mic);
                        }

                        mediaRecorder.SetVideoSource(VideoSource.Surface);
                        mediaRecorder.SetOutputFormat(OutputFormat.Mpeg4);
                        mediaRecorder.SetOutputFile(file);

                        //TODO: HO verify this
                        //mediaRecorder.SetVideoEncodingBitRate(10000000);

                        //TODO: HO verify this
                        //mediaRecorder.SetVideoFrameRate(30);
                        mediaRecorder.SetVideoFrameRate(fps ?? 30);

                        var maxVideoSize = ChooseMaxVideoSize(map.GetOutputSizes(Class.FromType(typeof(ImageReader))));
                        if (Resolution.Width != 0 && Resolution.Height != 0)
                            maxVideoSize = new((int)Resolution.Width, (int)Resolution.Height);
                        mediaRecorder.SetVideoSize(maxVideoSize.Width, maxVideoSize.Height);

                        //TODO: HO verify this
                        mediaRecorder.SetVideoEncodingBitRate(bitrate?.Invoke((int)Resolution.Height) ?? 10_000_000);

                        mediaRecorder.SetVideoEncoder(VideoEncoder.H264);

                        //HO changed
                        //mediaRecorder.SetAudioEncoder(AudioEncoder.Aac);
                        if (withAudio)
                        {
                            mediaRecorder.SetAudioEncoder(AudioEncoder.Aac);
                        }


                        IWindowManager windowManager = context.GetSystemService(Context.WindowService).JavaCast<IWindowManager>();


                        //HO changed
                        //int rotation = (int)windowManager.DefaultDisplay.Rotation;
                        if (!rotation.HasValue)
                        {
                            rotation = (int)windowManager.DefaultDisplay.Rotation;
                        }

                        int orientation = cameraView.Camera.Position == CameraPosition.Back ? orientation = ORIENTATIONS.Get(rotation.Value) : orientation = ORIENTATIONSFRONT.Get(rotation.Value);
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
        if (_logger.IsEnabled(LogLevel.Trace))
        {
            _logger.LogTrace($"{nameof(StartPreview)}: entered");
        }

        while (textureView.SurfaceTexture == null || !textureView.IsAvailable) Thread.Sleep(100);
        SurfaceTexture texture = textureView.SurfaceTexture;
        texture.SetDefaultBufferSize(videoSize.Width, videoSize.Height);

        previewBuilder = cameraDevice.CreateCaptureRequest(recording ? CameraTemplate.Record : CameraTemplate.Preview);

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
        if(recording && cameraView.TorchEnabled)
        {
            if(_logger.IsEnabled(LogLevel.Trace)) {
                _logger.LogTrace($"{nameof(StartPreview)}: {nameof(cameraView.TorchEnabled)}: Turning it on");
            }
            previewBuilder.Set(CaptureRequest.ControlAeMode, (int)ControlAEMode.On);
            previewBuilder.Set(CaptureRequest.FlashMode, cameraView.TorchEnabled ? (int)ControlAEMode.OnAutoFlash : (int)ControlAEMode.Off);
        }

        sessionCallback = new PreviewCaptureStateCallback(this);
        if (OperatingSystem.IsAndroidVersionAtLeast(28))
        {
            SessionConfiguration config = new((int)SessionType.Regular, surfaces, executorService, sessionCallback);
            cameraDevice.CreateCaptureSession(config);
        }
        else
        {
#pragma warning disable CS0618 // El tipo o el miembro están obsoletos
            cameraDevice.CreateCaptureSession(surfaces26, sessionCallback, null);
#pragma warning restore CS0618 // El tipo o el miembro están obsoletos
        }
        _logger.LogTrace("_previewStartedTcs?.TrySetResult()");
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
                //HO Added UpdateTorch //so when we reenter camera page it will always light up again
                UpdateTorch();
                //previewSession.SetRepeatingRequest(previewBuilder.Build(), null, null);
                if (recording)
                    mediaRecorder?.Start();
            }
            catch (CameraAccessException e)
            {
                e.PrintStackTrace();
            }
        }
    }

    object _previewStartedTcsLock = new object();
    TaskCompletionSource? _previewStartedTcs = null;

    internal async Task<CameraResult> StartCameraAsync(Microsoft.Maui.Graphics.Size PhotosResolution, int maxPhotoResolution)
    {
        var result = CameraResult.Success;
        if (initiated)
        {
            if (await CameraView.RequestPermissions())
            {
                if (started) StopCamera();
                if (cameraView.Camera != null)
                {
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
                            _logger.LogTrace($"{nameof(StartCameraAsync)}: {nameof(sensorRect)}: w: {sensorRect.Width()} h: {sensorRect.Height()}");
                        }
                        if (PhotosResolution.Width != 0 && PhotosResolution.Height != 0)
                        {
                            imgReaderSize = new((int)PhotosResolution.Width, (int)PhotosResolution.Height);
                        }
                        else
                        {   //HO changed to include non bursting sizes, otherwise  Samsung A34 gives images of size maxVideo ( which is less resolution)
                            var jpegSizes = GetAvailableJpegSizes(map);
                            imgReaderSize = jpegSizes.Last();
                            if (maxPhotoResolution < int.MaxValue)
                            {
                                //choose the first resolution that is less than maxResolution and has (about) same aspect as the last
                                var aspect = ((float)imgReaderSize.Width) / imgReaderSize.Height;
                                var minAspect = aspect - aspect / 10;
                                var maxAspect = aspect + aspect / 10;
                                {
                                    imgReaderSize = jpegSizes
                                        .LastOrDefault(x => ((float)x.Width) / x.Height >= minAspect && ((float)x.Width) / x.Height <= maxAspect && (x.Width * x.Height) < maxPhotoResolution);
                                }
                                /*
                                if (imgReaderSize == null)
                                { //none is small enough among jpeg, so lets try imageReader ones
                                    var imageReaderSizes = map.GetOutputSizes(Class.FromType(typeof(ImageReader))).OrderBy(x => x.Width * x.Height).ToArray();

                                    imgReaderSize = imageReaderSizes
                                        .LastOrDefault(x => ((float)x.Width) / x.Height >= minAspect && ((float)x.Width) / x.Height <= maxAspect && (x.Width * x.Height) < maxPhotoResolution);
                                }
                                */
                                if (imgReaderSize == null)
                                { //none is small enough, so lets take the first that matches aspect of camera sensor
                                    var outputSizes = map.GetOutputSizes(Class.FromType(typeof(ImageReader)));
                                    imgReaderSize = jpegSizes
                                    .FirstOrDefault(x => ((float)x.Width) / x.Height >= minAspect && ((float)x.Width) / x.Height <= maxAspect);
                                }
                                
                                if (imgReaderSize == null)
                                { //none is small enough so take the first
                                    imgReaderSize = jpegSizes.First();
                                }
                            }
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
                        }

                        if (OperatingSystem.IsAndroidVersionAtLeast(28))
                            cameraManager.OpenCamera(cameraView.Camera.DeviceId, executorService, stateListener);
                        else
                            cameraManager.OpenCamera(cameraView.Camera.DeviceId, stateListener, null);

                        _logger.LogTrace("PRE: await _previewStartedTcs.Task");
                        await _previewStartedTcs.Task;
                        _logger.LogTrace("POST: await _previewStartedTcs.Task");

                        timer.Start();

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
        return StartCameraAsync(cameraView.PhotosResolution, cameraView.MaxPhotoResolution);
    }

    internal CameraResult StopCamera()
    {
        CameraResult result = CameraResult.Success;
        lock(cameraDeviceLock)
        {
            if (initiated)
            {
                timer.Stop();
                try
                {
                    mediaRecorder?.Stop();
                    mediaRecorder?.Dispose();
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
    //private Bitmap TakeSnap_org()
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

    /*
    private Bitmap TakeSnap()
    //private Bitmap TakeSnap_new()
    {
    Bitmap bitmap = null;
        try
        {
            MainThread.InvokeOnMainThreadAsync(() => { 
                bitmap = textureView.GetBitmap(null); bitmap = textureView.Bitmap; 
            }).Wait();

            if (bitmap != null)
            {
                int oriWidth = bitmap.Width;
                int oriHeight = bitmap.Height;

                bitmap = Bitmap.CreateBitmap(bitmap, 0, 0, bitmap.Width, bitmap.Height, textureView.GetTransform(null), false);
                float xscale = (float)oriWidth / bitmap.Width;
                float yscale = (float)oriHeight / bitmap.Height;
                //bitmap = Bitmap.CreateBitmap(bitmap, Math.Abs(bitmap.Width - (int)((float)Width*xscale)) / 2, Math.Abs(bitmap.Height - (int)((float)Height * yscale)) / 2, Width, Height);
                var cameraViewSizeInPixels = GetCameraViewSizeInPixels();



                //HO check why these differs when starting scanbarcode page and if cameraViewSizeInPixels is really needed or we can go with textureView width Height
                bitmap = Bitmap.CreateBitmap(bitmap, 0, 0, (int)Math.Min(bitmap.Width, cameraViewSizeInPixels.Width), (int)Math.Min(bitmap.Height,cameraViewSizeInPixels.Height));
                Matrix matrix = null;
                if (textureView.ScaleX == -1)
                {
                    matrix ??= new();
                    matrix.PreScale(-1, 1);
                }
                if (matrix != null)
                {
                    //HO Recommended default is to set filter to 'true' as the cost of bilinear filtering is typically minimal and the improved image quality is significant.
                    bitmap = Bitmap.CreateBitmap(bitmap, 0, 0, bitmap.Width, bitmap.Height, matrix, true);
                }
            }
        }
        catch (Exception ex) {
            _logger.LogWarning(ex, nameof(TakeSnap));
        }
        return bitmap;
    }
    */

    private static Rect CalculateScalerRect(Rect sensorRect, Single zoomFactor)
    {
        Rect m = sensorRect;

        var ratio = 1 / zoomFactor;
        var sensorRectWidth = sensorRect.Right - sensorRect.Left;
        var sensorRectHeight = sensorRect.Bottom - sensorRect.Top;

        var w = (int)(sensorRectWidth * ratio);
        var newLeft = (sensorRectWidth - w) / 2;
        var h = (int)(sensorRectHeight * ratio);
        var newTop = (sensorRectHeight - h) / 2;
        return new Rect(newLeft, newTop, w + newLeft, h + newTop);
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

                    if (_logger.IsEnabled(LogLevel.Trace))
                    {
                        _logger.LogTrace($"{nameof(TakePhotoAsync)}: 20: TextureView: {textureView.ToString()} ScaleX:{textureView.ScaleX} ScaleY: {textureView.ScaleY}");
                    }
                    if (textureView.ScaleX == -1 || imageFormat != ImageFormat.JPEG)
                    {
                        Bitmap bitmap = BitmapFactory.DecodeByteArray(capturePhoto, 0, capturePhoto.Length);
                        if (textureView.ScaleX == -1)
                        {
                            Matrix matrix = new();
                            matrix.PreRotate(rotation.Value);
                            matrix.PostScale(-1, 1);
                            bitmap = Bitmap.CreateBitmap(bitmap, 0, 0, bitmap.Width, bitmap.Height, matrix, false);
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

                var resolution = (float)(bitmap.Width * bitmap.Height);
                if (resolution > cameraView.MaxPhotoResolution)
                {   //HO honor maxResolution
                    matrix ??= new();
                    float ratio = (float)Math.Sqrt((float)cameraView.MaxPhotoResolution / resolution);
                    matrix.PostScale(ratio, ratio);
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
        if (cameraView.Camera != null && cameraView.Camera.HasFlashUnit)
        {
            if (_logger.IsEnabled(LogLevel.Trace)) {
                _logger.LogTrace($"{nameof(UpdateTorch)}: {nameof(cameraView.TorchEnabled)}: Turning it on");
            }
            if (started)
            {
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
        if (previewSession != null && previewBuilder != null && cameraView.Camera != null)
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

    Size GetCameraViewSizeInPixels()
    {
        var cameraViewWidthInPixels = (int)Math.Round(cameraView.Width * DeviceDisplay.Current.MainDisplayInfo.Density);
        var cameraViewHeightInPixels = (int)Math.Round(cameraView.Height * DeviceDisplay.Current.MainDisplayInfo.Density);
        return new Size(cameraViewWidthInPixels, cameraViewHeightInPixels);
    }


    private Size ChooseVideoSize(Size[] choices)
    {
        bool swapped = IsDimensionSwapped();

        //from smaller to larger
        choices = choices.OrderBy(x => x.Width * x.Height).ToArray();

        Size result = choices[0];


        var cameraViewSizeInPixels = GetCameraViewSizeInPixels();

        var cameraViewSizeInPixels_Width = cameraViewSizeInPixels.Width;
        var cameraViewSizeInPixels_Height = cameraViewSizeInPixels.Height;

        if (swapped)
        {
            cameraViewSizeInPixels_Width = cameraViewSizeInPixels.Height;
            cameraViewSizeInPixels_Height = cameraViewSizeInPixels.Width;
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

        {
            _logger.LogTrace($"{nameof(ChooseVideoSize)}: selected: w:{result?.Width} h:{result?.Height}");
        }
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


        var cameraViewSizeInPixels = GetCameraViewSizeInPixels();

        RectF viewRect = new(0, 0, (float)cameraViewSizeInPixels.Width, (float)cameraViewSizeInPixels.Height);
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
                    (float)cameraViewSizeInPixels.Height / (isSwapped ? videoWidth : videoHeight),
                    (float)cameraViewSizeInPixels.Width / (isSwapped ? videoHeight : videoWidth));
        }
        else
        {
            scale = Math.Max(
                    (float)cameraViewSizeInPixels.Height / (isSwapped ? videoWidth :  videoHeight),
                    (float)cameraViewSizeInPixels.Width / (isSwapped ? videoHeight : videoWidth));
        }
        txform.PostScale(scale, scale, centerX, centerY);

        //txform.PostScale(scaleX, scaleY, centerX, centerY);
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
        textureView.SetTransform(txform);
    }

    protected override async void OnConfigurationChanged(Configuration newConfig)
    {
        base.OnConfigurationChanged(newConfig);
        if (started && !recording)
            await StartCameraAsync(cameraView.PhotosResolution, cameraView.MaxPhotoResolution);
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
        public MyCameraStateCallback(MauiCameraView camView, ILogger logger)
        {
            cameraView = camView;
            _logger = logger;
        }
        public override void OnOpened(CameraDevice camera)
        {
            if (_logger.IsEnabled(LogLevel.Trace)) {
                _logger.LogTrace($"{nameof(MyCameraStateCallback)}: {nameof(OnOpened)}: entered");
            }  
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
        public PreviewCaptureStateCallback(MauiCameraView camView)
        {
            cameraView = camView;
        }
        public override void OnConfigured(CameraCaptureSession session)
        {
            cameraView.previewSession = session;
            cameraView.UpdatePreview();

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
}


