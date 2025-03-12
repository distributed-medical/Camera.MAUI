﻿#if IOS || MACCATALYST
using AVFoundation;
using CoreAnimation;
using CoreFoundation;
using CoreGraphics;
using CoreImage;
using CoreMedia;
using CoreVideo;
using Foundation;
using MediaPlayer;
using Microsoft.Extensions.Logging;
using System.IO;
using UIKit;
using VideoToolbox;

namespace Camera.MAUI.Platforms.Apple;

internal class MauiCameraView : UIView, IAVCaptureVideoDataOutputSampleBufferDelegate, IAVCaptureFileOutputRecordingDelegate, IAVCapturePhotoCaptureDelegate
{
    private AVCaptureDevice[] camDevices;
    private AVCaptureDevice[] micDevices;
    private readonly CameraView cameraView;
    private readonly AVCaptureVideoPreviewLayer PreviewLayer;
    private readonly AVCaptureVideoDataOutput videoDataOutput;
    private AVCaptureMovieFileOutput recordOutput;
    private readonly AVCapturePhotoOutput photoOutput;
    private AVCapturePhotoOutput snapshotOutput;
    private readonly AVCaptureSession captureSession;
    private AVCaptureDevice captureDevice;
    private AVCaptureDevice micDevice;
    private AVCaptureInput captureInput = null;
    private AVCaptureInput micInput = null;
    private bool started = false;
    private CIImage lastCapture;
    private readonly object lockCapture = new();
    private readonly DispatchQueue cameraDispacher;
    private int frames = 0, currentFrames = 0;
    private bool initiated = false;
    private bool snapping = false;
    private bool photoTaken = false;
    private bool photoError = false;
    private UIImage photo;
    private readonly NSObject orientationObserver;

    readonly ILogger _logger;
    readonly Action<string> _logger_LogTrace = null;

    public MauiCameraView(CameraView cameraView)
    {
        _logger = IPlatformApplication.Current.Services.GetService<ILogger<MauiCameraView>>();
        if (_logger.IsEnabled(LogLevel.Trace))
        {
            _logger_LogTrace = (str) => _logger.LogTrace(str);
        }


        this.cameraView = cameraView;

        captureSession = new AVCaptureSession
        {   
            SessionPreset = AVCaptureSession.PresetPhoto
        };
        PreviewLayer = new(captureSession)
        {
                VideoGravity = cameraView.AspectFitPreview ? AVLayerVideoGravity.ResizeAspect : AVLayerVideoGravity.ResizeAspectFill
        };

        Layer.AddSublayer(PreviewLayer);
        videoDataOutput = new AVCaptureVideoDataOutput();
        var videoSettings = NSDictionary.FromObjectAndKey(
            new NSNumber((int)CVPixelFormatType.CV32BGRA),
            CVPixelBuffer.PixelFormatTypeKey);
        videoDataOutput.WeakVideoSettings = videoSettings;
        videoDataOutput.AlwaysDiscardsLateVideoFrames = true;
        photoOutput = new AVCapturePhotoOutput();
        cameraDispacher = new DispatchQueue("CameraDispacher");

        videoDataOutput.SetSampleBufferDelegate(this, cameraDispacher);
        orientationObserver = NSNotificationCenter.DefaultCenter.AddObserver(UIDevice.OrientationDidChangeNotification, OrientationChanged);
        InitDevices();
    }
    private void OrientationChanged(NSNotification notification)
    {
        LayoutSubviews();
    }
    private void InitDevices()
    {
        if (!initiated)
        {
            try
            {
                var deviceDescoverySession = AVCaptureDeviceDiscoverySession.Create(new AVCaptureDeviceType[] {
                    AVCaptureDeviceType.BuiltInTripleCamera, 
                    AVCaptureDeviceType.BuiltInDualCamera,
                    AVCaptureDeviceType.BuiltInWideAngleCamera,
                }, AVMediaTypes.Video, AVCaptureDevicePosition.Unspecified);
                camDevices = deviceDescoverySession.Devices;
                cameraView.Cameras.Clear();
                foreach (var device in camDevices)
                {
                    CameraPosition position = device.Position switch
                    {
                        AVCaptureDevicePosition.Back => CameraPosition.Back,
                        AVCaptureDevicePosition.Front => CameraPosition.Front,
                        _ => CameraPosition.Unknow
                    };
                    //HO they will come in order as requested so if has triple camera it comes first
                    //HO A triple camera will automatically change camera depending on distance this will automatically enable macro for instance
                    //HO we add them if we dont have one for that camera position yet
                    if(!cameraView.Cameras.Any(x => x.Position == position))
                    {
                        cameraView.Cameras.Add(new CameraInfo
                        {
                            Name = device.LocalizedName,
                            DeviceId = device.UniqueID,
                            Position = position,
                            HasFlashUnit = device.FlashAvailable,
                            MinZoomFactor = (float)device.MinAvailableVideoZoomFactor,
                            MaxZoomFactor = (float)device.MaxAvailableVideoZoomFactor,
                            HorizontalViewAngle = device.ActiveFormat.VideoFieldOfView * MathF.PI / 180f,
                            AvailableResolutions = new() { new(3840, 2160), new(1920, 1080), new(1280, 720), new(640, 480), new(352, 288) }
                        });
                    }
                }
                deviceDescoverySession.Dispose();
                var aSession = AVCaptureDeviceDiscoverySession.Create(new AVCaptureDeviceType[] { AVCaptureDeviceType.BuiltInMicrophone }, AVMediaTypes.Audio, AVCaptureDevicePosition.Unspecified);
                micDevices = aSession.Devices;
                cameraView.Microphones.Clear();
                foreach (var device in micDevices)
                    cameraView.Microphones.Add(new MicrophoneInfo { Name = device.LocalizedName, DeviceId = device.UniqueID });
                aSession.Dispose();
                initiated = true;
                cameraView.RefreshDevices();
            }
            catch
            {
            }
        }
    }

    public async Task<CameraResult> StartRecordingAsync(string file, Size Resolution, OtherRecordingParameters otherRecordingParameters)
    {
        CameraResult result = CameraResult.Success;
        if (initiated)
        {
            var withAudio = otherRecordingParameters?.WithAudio ?? true;

            if (started) StopCamera();

            if (await CameraView.RequestPermissions(withAudio))
            {
                if (cameraView.Camera != null && cameraView.Microphone != null && captureSession != null)
                {
                    try
                    {

                        //HO changed
                        /*
                        captureSession.SessionPreset = Resolution.Width switch
                        {
                            352 => AVCaptureSession.Preset352x288,
                            640 => AVCaptureSession.Preset640x480,
                            1280 => AVCaptureSession.Preset1280x720,
                            1920 => AVCaptureSession.Preset1920x1080,
                            3840 => AVCaptureSession.Preset3840x2160,
                            _ => AVCaptureSession.PresetPhoto
                        };
                        */
                        //HO SelectBestRecordingResolution will use ActiveFormat, AVCaptureSession.PresetInputPriority means we are using ActiveFormat
                        captureSession.SessionPreset = AVCaptureSession.PresetInputPriority;

                        frames = 0;
                        captureDevice = camDevices.First(d => d.UniqueID == cameraView.Camera.DeviceId);
                        ForceAutoFocus();
                        captureInput = new AVCaptureDeviceInput(captureDevice, out var err);


                        captureSession.AddInput(captureInput);
                        //HO changed
                        //captureSession.AddInput(micInput);
                        if (withAudio)
                        {
                            micDevice = micDevices.First(d => d.UniqueID == cameraView.Microphone.DeviceId);
                            micInput = new AVCaptureDeviceInput(micDevice, out err);

                            captureSession.AddInput(micInput);
                        }

                        //HO added 2024-06-20 BEGIN
                        snapshotOutput = new AVCapturePhotoOutput();
                        snapshotOutput.IsHighResolutionCaptureEnabled = true;
                        snapshotOutput.MaxPhotoQualityPrioritization = AVCapturePhotoQualityPrioritization.Quality;
                        captureSession.AutomaticallyConfiguresCaptureDeviceForWideColor = false;


                        if (captureSession.CanAddOutput(snapshotOutput)) {
                            captureSession.AddOutput(snapshotOutput);
                        }
                        else
                        {
                            Debug("failed to add snapshot output");
                            return result = CameraResult.AccessError;
                        }
                        //HO added 2024-06-20 END



                        captureSession.AddOutput(videoDataOutput);
                        recordOutput = new AVCaptureMovieFileOutput();



                        captureSession.AddOutput(recordOutput);
                        var movieFileOutputConnection = recordOutput.Connections[0];
                        
                        var currentDeviceOrientation = UIDevice.CurrentDevice.Orientation;
                        if (otherRecordingParameters.RotationRelativeToPortrait != null)
                        {
                            currentDeviceOrientation = otherRecordingParameters.RotationRelativeToPortrait switch
                            {
                                90 => UIDeviceOrientation.LandscapeLeft,
                                180 => UIDeviceOrientation.PortraitUpsideDown,
                                270 => UIDeviceOrientation.LandscapeRight,
                                _ => UIDeviceOrientation.Portrait
                            };
                        }
                        
                        movieFileOutputConnection.VideoOrientation = (AVCaptureVideoOrientation)currentDeviceOrientation;
                        

                        if(!SelectBestRecordingResolution(captureDevice, recordOutput, movieFileOutputConnection,  Resolution, otherRecordingParameters))
                        {
                            return CameraResult.NoVideoFormatsAvailable;
                        }


                        //HO Added
                        if(movieFileOutputConnection.ActiveVideoStabilizationMode == AVCaptureVideoStabilizationMode.Off)
                        {
                            movieFileOutputConnection.PreferredVideoStabilizationMode = AVCaptureVideoStabilizationMode.Standard;
                        }
                        captureSession.StartRunning();
                        if (File.Exists(file)) File.Delete(file);
                        
                        UpdateMirroredImage();
                        SetZoomFactor(cameraView.ZoomFactor);

                        //HO  changed let captureSession run a while or we get dark video at start while camera is measuring light 180ms seems ok
                        await Task.Delay(180);

                        //HO  changed moved StartRecordingToOutputFile below UpdateMirroredImage and SetZoomFactor
                        recordOutput.StartRecordingToOutputFile(NSUrl.FromFilename(file), this);
                        started = true;
                    }
                    catch(Exception ex)
                    {
                        _ = ex;
                        result = CameraResult.AccessError;
                    }
                }
                else
                    result = CameraResult.AccessError;
            }
            else
                result = CameraResult.AccessDenied;
        }
        else
            result = CameraResult.NotInitiated;
        return result;
    }
    public Task<CameraResult> StopRecordingAsync()
    {
        return StartCameraAsync(cameraView.PhotosResolution);
    }


    public async Task<CameraResult> StartCameraAsync(Size PhotosResolution)
    {
        CameraResult result = CameraResult.Success;
        if (initiated)
        {
            if (started) StopCamera();
            if (await CameraView.RequestPermissions())
            {
                if (cameraView.Camera != null && captureSession != null)
                {
                    try
                    {
                        captureSession.SessionPreset = PhotosResolution.Width switch
                        {
                            352 => AVCaptureSession.Preset352x288,
                            640 => AVCaptureSession.Preset640x480,
                            1280 => AVCaptureSession.Preset1280x720,
                            1920 => AVCaptureSession.Preset1920x1080,
                            3840 => AVCaptureSession.Preset3840x2160,
                            _ => AVCaptureSession.PresetPhoto
                        };
                        frames = 0;
                        captureDevice = camDevices.First(d => d.UniqueID == cameraView.Camera.DeviceId);
                        ForceAutoFocus();
                        captureInput = new AVCaptureDeviceInput(captureDevice, out var err);
                        captureSession.AddInput(captureInput);
                        captureSession.AddOutput(videoDataOutput);
                        captureSession.AddOutput(photoOutput);
                        captureSession.StartRunning();
                        UpdateMirroredImage();
                        SetZoomFactor(cameraView.ZoomFactor);
                        started = true;
                    }
                    catch
                    {
                        result = CameraResult.AccessError;
                    }
                }
                else
                    result = CameraResult.AccessError;
            }
            else
                result = CameraResult.AccessDenied;
        }else
            result = CameraResult.NotInitiated;
        return result;
    }
    public CameraResult StopCamera()
    {
        CameraResult result = CameraResult.Success;
        if (initiated)
        {
            try
            {
                if (captureSession != null)
                {
                    if (captureSession.Running)
                        captureSession.StopRunning();
                    if (recordOutput != null)
                    {
                        recordOutput.StopRecording();
                        captureSession.RemoveOutput(recordOutput);
                        recordOutput.Dispose();
                        recordOutput = null;
                    }
                    foreach (var output in captureSession.Outputs)
                        captureSession.RemoveOutput(output);
                    foreach (var input in captureSession.Inputs)
                    {
                        captureSession.RemoveInput(input);
                        input.Dispose();
                    }
                }
                started = false;
            }
            catch
            {
                result = CameraResult.AccessError;
            }
        }else
            result = CameraResult.NotInitiated;

        return result;
    }
    public void DisposeControl()
    {
        if (started) StopCamera();
        NSNotificationCenter.DefaultCenter.RemoveObserver(orientationObserver);
        PreviewLayer?.Dispose();
        captureSession?.Dispose();
        Dispose();
    }
    public void UpdateMirroredImage()
    {
        if (cameraView != null && PreviewLayer.Connection != null)
        {
            if (PreviewLayer.Connection.AutomaticallyAdjustsVideoMirroring)
                PreviewLayer.Connection.AutomaticallyAdjustsVideoMirroring = false;
            if (cameraView.MirroredImage)
                PreviewLayer.Connection.VideoMirrored = true;
            else
                PreviewLayer.Connection.VideoMirrored = false;

            UpdateTorch();
        }
    }
    internal void SetZoomFactor(float zoom)
    {
        if (cameraView.Camera != null && captureDevice != null)
        {
            captureDevice.LockForConfiguration(out NSError error);
            if (error == null)
            {
                captureDevice.VideoZoomFactor = Math.Clamp(zoom, cameraView.Camera.MinZoomFactor, cameraView.Camera.MaxZoomFactor);
                captureDevice.UnlockForConfiguration();
            }
        }
    }
    internal void ForceAutoFocus()
    {
        if (cameraView.Camera != null && captureDevice != null && captureDevice.IsFocusModeSupported(AVCaptureFocusMode.AutoFocus))
        {
            captureDevice.LockForConfiguration(out NSError error);
            if (error == null)
            {
                if (captureDevice.IsFocusModeSupported(AVCaptureFocusMode.ContinuousAutoFocus))
                    captureDevice.FocusMode = AVCaptureFocusMode.ContinuousAutoFocus;
                else
                    captureDevice.FocusMode = AVCaptureFocusMode.AutoFocus;
                captureDevice.UnlockForConfiguration();
            }
        }
    }
    public void UpdateTorch()
    {
        if (captureDevice != null && cameraView != null)
        {
            captureDevice.LockForConfiguration(out NSError error);
            if (error == null)
            {
                if (captureDevice.HasTorch && captureDevice.TorchAvailable)
                    captureDevice.TorchMode = cameraView.TorchEnabled ? AVCaptureTorchMode.On : AVCaptureTorchMode.Off;
                captureDevice.UnlockForConfiguration();
            }
        }
    }

    internal async Task<Stream> TakePhotoAsync(ImageFormat imageFormat, int? rotation)
    {
        return await TakePhotoAsync(photoOutput, imageFormat, rotation);
    }

    private async Task<Stream> TakePhotoAsync(AVCapturePhotoOutput photoOrSnapshotOutput, ImageFormat imageFormat, int? rotation)
    {
        photoError = photoTaken = false;
        var photoSettings = AVCapturePhotoSettings.Create();


        //HO Added
        if (photoOrSnapshotOutput == snapshotOutput)
        { //This gives snapshot a higher resolution
            photoSettings.IsHighResolutionPhotoEnabled = snapshotOutput.IsHighResolutionCaptureEnabled;
            photoSettings.PhotoQualityPrioritization = snapshotOutput.MaxPhotoQualityPrioritization;
        }


        photoSettings.FlashMode = cameraView.FlashMode switch
        {
            FlashMode.Auto => cameraView.TorchEnabled ? AVCaptureFlashMode.Off : AVCaptureFlashMode.Auto,
            FlashMode.Enabled => cameraView.TorchEnabled ? AVCaptureFlashMode.Off : AVCaptureFlashMode.On,
            //FlashMode.Auto => AVCaptureFlashMode.Auto,
            //FlashMode.Enabled => AVCaptureFlashMode.On,
            _ => AVCaptureFlashMode.Off
        };
        photoOrSnapshotOutput.CapturePhoto(photoSettings, this);
        while(!photoTaken && !photoError) await Task.Delay(50);
        if (photoError || photo == null)
            return null;
        else
        {
            var currentDeviceOrientation = UIDevice.CurrentDevice.Orientation;

            //HO changed
            if (rotation != null)
            {
                currentDeviceOrientation = rotation switch { 
                    90 => UIDeviceOrientation.LandscapeLeft,
                    180 => UIDeviceOrientation.PortraitUpsideDown,
                    270 => UIDeviceOrientation.LandscapeRight,
                    _ => UIDeviceOrientation.Portrait
                };
            }
            UIImageOrientation orientation = currentDeviceOrientation switch
            {
                UIDeviceOrientation.LandscapeRight => cameraView.Camera?.Position == CameraPosition.Back ? UIImageOrientation.Down : UIImageOrientation.Up,
                UIDeviceOrientation.LandscapeLeft => cameraView.Camera?.Position == CameraPosition.Back ? UIImageOrientation.Up : UIImageOrientation.Down,
                UIDeviceOrientation.PortraitUpsideDown => UIImageOrientation.Left,
                _ => UIImageOrientation.Right
            };
            if (photo.Orientation != orientation)
                photo = UIImage.FromImage(photo.CGImage, photo.CurrentScale, orientation);
            MemoryStream stream = new();
            switch (imageFormat)
            {
                case ImageFormat.JPEG:
                    photo.AsJPEG().AsStream().CopyTo(stream);
                    break;
                default:
                    photo.AsPNG().AsStream().CopyTo(stream);
                    break;
            }
            stream.Position = 0;
            return stream;
        }
    }
    public ImageSource GetSnapShot(ImageFormat imageFormat, bool auto = false)
    {
        ImageSource result = null;

        if (started && lastCapture != null && !snapping)
        {
            MainThread.InvokeOnMainThreadAsync(() =>
            {
                snapping = true;
                try
                {
                    lock (lockCapture)
                    {
                        var ciContext = new CIContext();
                        CGImage cgImage = ciContext.CreateCGImage(lastCapture, lastCapture.Extent);
                        UIImageOrientation orientation = UIDevice.CurrentDevice.Orientation switch
                        {
                            UIDeviceOrientation.LandscapeRight => UIImageOrientation.Down,
                            UIDeviceOrientation.LandscapeLeft => UIImageOrientation.Up,
                            UIDeviceOrientation.PortraitUpsideDown => UIImageOrientation.Left,
                            _ => UIImageOrientation.Right
                        };
                        var image = UIImage.FromImage(cgImage, UIScreen.MainScreen.Scale, orientation);
                        var image2 = CropImage(image);
                        MemoryStream stream = new();
                        switch (imageFormat)
                        {
                            case ImageFormat.JPEG:
                                image2.AsJPEG().AsStream().CopyTo(stream);
                                break;
                            default:
                                image2.AsPNG().AsStream().CopyTo(stream);
                                break;
                        }
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

                    }
                }
                catch
                {
                }
                snapping = false;
            }).Wait();
        }

        return result;
    }
    public bool SaveSnapShot(ImageFormat imageFormat, string SnapFilePath, int? rotation)
    {
        bool result = true;

        if (started && lastCapture != null)
        {
            if (File.Exists(SnapFilePath)) File.Delete(SnapFilePath);      
            
            MainThread.InvokeOnMainThreadAsync(async () =>
            {
                try
                {
                    /*
                    lock (lockCapture)
                    {
                        var ciContext = new CIContext();
                        CGImage cgImage = ciContext.CreateCGImage(lastCapture, lastCapture.Extent);
                        var currentDeviceOrientation = UIDevice.CurrentDevice.Orientation;
                        //HO changed
                        if (rotation != null)
                        {
                            currentDeviceOrientation = (rotation) switch
                            {
                                90 => UIDeviceOrientation.LandscapeLeft,
                                180 => UIDeviceOrientation.PortraitUpsideDown,
                                270 => UIDeviceOrientation.LandscapeRight,
                                _ => UIDeviceOrientation.Portrait
                            };
                        }

                        UIImageOrientation orientation = currentDeviceOrientation switch
                        {
                            UIDeviceOrientation.LandscapeRight => UIImageOrientation.Right,
                            UIDeviceOrientation.LandscapeLeft => UIImageOrientation.Left,
                            UIDeviceOrientation.PortraitUpsideDown => UIImageOrientation.Down,
                            _ => UIImageOrientation.Up
                        };
                        var image = UIImage.FromImage(cgImage, UIScreen.MainScreen.Scale, orientation);
                        var image2 = CropImage(image);
                        switch (imageFormat)
                        {
                            case ImageFormat.PNG:
                                image2.AsPNG().Save(NSUrl.FromFilename(SnapFilePath), true);
                                break;
                            case ImageFormat.JPEG:
                                image2.AsJPEG().Save(NSUrl.FromFilename(SnapFilePath), true);
                                break;
                        }
                    }
                    */
                    using (var fileStream = new FileStream(SnapFilePath, FileMode.Create, FileAccess.Write, FileShare.ReadWrite))
                    {
                        var stream = await TakePhotoAsync(snapshotOutput, imageFormat, rotation);
                        await stream.CopyToAsync(fileStream);
                    }
                }
                catch
                {
                    result = false;
                }
            }).Wait();
        }
        else
            result = false;
        return result;
    }
    public UIImage CropImage(UIImage originalImage)
    {
        nfloat x, y, width, height;

        if (originalImage.Size.Width <= originalImage.Size.Height)
        {
            width = originalImage.Size.Width;
            height = (Frame.Size.Height * originalImage.Size.Width) / Frame.Size.Width;
        }
        else
        {
            height = originalImage.Size.Height;
            width = (Frame.Size.Width * originalImage.Size.Height) / Frame.Size.Height;
        }

        x = (nfloat)((originalImage.Size.Width - width) / 2.0);
        y = (nfloat)((originalImage.Size.Height - height) / 2.0);

        UIGraphics.BeginImageContextWithOptions(originalImage.Size, false, 1);
        if (cameraView.MirroredImage)
        {
            var context = UIGraphics.GetCurrentContext();
            context.ScaleCTM(-1, 1);
            context.TranslateCTM(-originalImage.Size.Width, 0);
        }
        originalImage.Draw(new CGPoint(0, 0));
        UIImage croppedImage = UIImage.FromImage(UIGraphics.GetImageFromCurrentImageContext().CGImage.WithImageInRect(new CGRect(new CGPoint(x, y), new CGSize(width, height))));
        UIGraphics.EndImageContext();

        return croppedImage;
    }
    private void ProccessQR()
    {
        MainThread.BeginInvokeOnMainThread(() =>
        {
            try
            {
                UIImage image2;
                lock (lockCapture)
                {
                    var ciContext = new CIContext();
                    CGImage cgImage = ciContext.CreateCGImage(lastCapture, lastCapture.Extent);
                    var image = UIImage.FromImage(cgImage, UIScreen.MainScreen.Scale, UIImageOrientation.Right);
                    image2 = CropImage(image);
                }
                cameraView.DecodeBarcode(image2);
            }
            catch
            {
            }
        });
    }
    private void ProcessImage(CIImage capture)
    {        
        new Task(() =>
        {
            lock (lockCapture)
            {
                lastCapture?.Dispose();
                lastCapture = capture;
            }
            if (!snapping && cameraView.AutoSnapShotSeconds > 0 && (DateTime.Now - cameraView.lastSnapshot).TotalSeconds >= cameraView.AutoSnapShotSeconds)
                cameraView.RefreshSnapshot(GetSnapShot(cameraView.AutoSnapShotFormat, true));
            else if (cameraView.BarCodeDetectionEnabled && currentFrames >= cameraView.BarCodeDetectionFrameRate)
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
                    currentFrames = 0;
                    lock (cameraView.currentThreadsLocker) cameraView.currentThreads--;
                }
            }
        }).Start();
    }

    [Export("captureOutput:didOutputSampleBuffer:fromConnection:")]
    public void DidOutputSampleBuffer(AVCaptureOutput captureOutput, CMSampleBuffer sampleBuffer, AVCaptureConnection connection)
    {
        frames++;
        currentFrames++;
        if (frames >= 12 || (cameraView.BarCodeDetectionEnabled && currentFrames >= cameraView.BarCodeDetectionFrameRate))
        {
            var capture = CIImage.FromImageBuffer(sampleBuffer.GetImageBuffer());
            ProcessImage(capture);
            sampleBuffer.Dispose();
            frames = 0;
            GC.Collect();
        }
        else
        {
            sampleBuffer?.Dispose();
        }
    }
    [Export("captureOutput:didFinishProcessingPhotoSampleBuffer:previewPhotoSampleBuffer:resolvedSettings:bracketSettings:error:")]
    void DidFinishProcessingPhoto(AVCapturePhotoOutput captureOutput, CMSampleBuffer photoSampleBuffer, CMSampleBuffer previewPhotoSampleBuffer, AVCaptureResolvedPhotoSettings resolvedSettings, AVCaptureBracketedStillImageSettings bracketSettings, NSError error)
    {
        if (photoSampleBuffer == null)
        {
            photoError = true;
            return;
        }

        NSData imageData = AVCapturePhotoOutput.GetJpegPhotoDataRepresentation(photoSampleBuffer, previewPhotoSampleBuffer);

        photo = new UIImage(imageData);

        photoTaken = true;
    }


    CGImage Rotate90Degrees(CGImage cgImageRef)
    {
        int cgImageWidth = (int)cgImageRef.Width;
        int cgImageHeight = (int)cgImageRef.Height;

        var colorSpace = CGColorSpace.CreateDeviceRGB();
        var bytesPerPixel = 4;

        int targetWidth = cgImageHeight;
        int targetHeight = cgImageWidth;

        var bytesPerRow = bytesPerPixel * targetWidth;
        var bitsPerComponent = 8;

        var context = new CGBitmapContext(new nint(0), targetWidth, targetHeight, bitsPerComponent, bytesPerRow, colorSpace,
            CoreGraphics.CGImageAlphaInfo.PremultipliedLast);
        context.RotateCTM(new nfloat(-(Math.PI + Math.PI / 2)));
        context.TranslateCTM(0, -targetWidth);
        context.DrawImage(new CGRect(0, 0, cgImageWidth, cgImageHeight), cgImageRef);
        var rotatedCGImage = context.ToImage();
        return rotatedCGImage;
    }


    public override void LayoutSubviews()
    {
        base.LayoutSubviews();
        //HO commented away we always do portrait
        /*
        CATransform3D transform = CATransform3D.MakeRotation(0, 0, 0, 1.0f);

        switch (UIDevice.CurrentDevice.Orientation)
        {
            case UIDeviceOrientation.Portrait:
                transform = CATransform3D.MakeRotation(0, 0, 0, 1.0f);
                break;
            case UIDeviceOrientation.PortraitUpsideDown:
                transform = CATransform3D.MakeRotation((nfloat)Math.PI, 0, 0, 1.0f);
                break;
            case UIDeviceOrientation.LandscapeLeft:
                var rotation = cameraView.Camera?.Position == CameraPosition.Back ? -Math.PI / 2 : Math.PI / 2;
                transform = CATransform3D.MakeRotation((nfloat)rotation, 0, 0, 1.0f);
                break;
            case UIDeviceOrientation.LandscapeRight:
                var rotation2 = cameraView.Camera?.Position == CameraPosition.Back ? Math.PI / 2 : -Math.PI /2;
                transform = CATransform3D.MakeRotation((nfloat)rotation2, 0, 0, 1.0f);
                break;
        }

        PreviewLayer.Transform = transform;
        */
        PreviewLayer.Frame = Layer.Bounds;
    }

    public void FinishedRecording(AVCaptureFileOutput captureOutput, NSUrl outputFileUrl, NSObject[] connections, NSError error)
    {
        
    }


    void Debug(string str)
    {
        System.Diagnostics.Debug.WriteLine(str);
    }

    //from capture ios
    float foundMaxFramerate = 0.0f;
    int desiredVideoResolution = 0;
    float desiredVideoFramerate = 0.0f;
    AVCaptureDeviceFormat foundVideoFormat = null;
    int selectedVideoResolution = 0;
    float selectedVideoFramerate = 0.0f;

    const string _strH264 = "h264";

    AVVideoCodecType? CodecFromString(string codec)
    {
        AVVideoCodecType? retval = codec switch
        {
            _strH264 => AVVideoCodecType.H264,
            "hevc" => AVVideoCodecType.Hevc,
            _ => AVVideoCodecType.H264,
        };
        return retval;
    }



    bool SelectBestRecordingResolution(AVCaptureDevice videoCaptureDevice, AVCaptureMovieFileOutput videoOutput, AVCaptureConnection videoOutputConnection,  Size wantedResolution, OtherRecordingParameters otherRecordingParameters)
    {
        bool retVal = true;
        // Resolve the desired video settings, defaulting to 720p30 whenever something seems broken
        desiredVideoResolution = (int)wantedResolution.Height;
        desiredVideoFramerate = otherRecordingParameters?.Fps ?? 30;


        desiredVideoResolution = Math.Min(desiredVideoResolution, 2160); // Higher blows up when trying to record

        // Format selection. Guiding principles:
        // - try to find a format that matches the requested vertical resolution
        // - if we can't find an exact match, pick the highest resolution that is still lower than the requested one
        // - never pick a format with a higher vertical resolution than requested
        // - among the matches, pick the one with the highest frame rate
        // - but also optimize for HRSI (still resolution), never picking a format with a lower HRSI once we've found one that is acceptable...

        var foundMaxResolution = 0;
        var foundFoV = 0.0f;
        var foundStillResolution = 0;

        if (foundVideoFormat == null)
        {

            foreach (var format in videoCaptureDevice.Formats)
            {
                var formatFormatDescription = format.FormatDescription as CoreMedia.CMVideoFormatDescription;
                var dimHeight = 0;
                if (formatFormatDescription != null) {
                    dimHeight = formatFormatDescription.Dimensions.Height;
                }
                //var dimHeight = ((CoreMedia.CMVideoFormatDescription)(format.FormatDescription)).Dimensions.Height.;

                bool considerThisFormat = true;

                if (foundMaxResolution > 0 && dimHeight > desiredVideoResolution) {
                    // We've found a format that works, we should not use one with higher resolution than requested.
                    Debug($"skipping format with excessive resolution:{format}");
                    considerThisFormat = false;
                }


                if (considerThisFormat)
                {
                    foreach (var frr in format.VideoSupportedFrameRateRanges)
                    {
                        var stillResolution = format.HighResolutionStillImageDimensions.Width * format.HighResolutionStillImageDimensions.Height;


                        if (dimHeight == foundMaxResolution)
                        {
                            // We've already found a format with the desired vertical resolution, let's see if this one is an improvement.
                            // We don't want to reduce our field of view
                            if (considerThisFormat && format.VideoFieldOfView < foundFoV)
                            {
                                Debug($"skipping format with reduced FoV:{format}, {frr}");
                                considerThisFormat = false;
                            }

                            // We don't want to reduce the still resolution
                            if (considerThisFormat && stillResolution < foundStillResolution)
                            {
                                Debug($"skipping format with reduced still resolution: {format}, {frr}");
                                considerThisFormat = false;
                            }

                            // We prefer non-binned video
                            if (considerThisFormat && format.VideoBinned)
                            {
                                Debug($"skipping format with binned video: {format}, {frr}");
                                considerThisFormat = false;
                            }

                            // Skip formats with a lower frame rate, unless it also increases our still resolution
                            if (considerThisFormat && (frr.MaxFrameRate < foundMaxFramerate) && (frr.MaxFrameRate < desiredVideoFramerate) && (stillResolution <= foundStillResolution))
                            {
                                Debug($"skipping format with lower frame rate: {format}, {frr}");
                                considerThisFormat = false;
                            }

                            // Don't downgrade from 420f to 420v
                            if (considerThisFormat && (format.FormatDescription.MediaSubType == (uint)CVPixelFormatType.CV420YpCbCr8BiPlanarVideoRange)
                                && (foundVideoFormat?.FormatDescription.MediaSubType == (uint)CVPixelFormatType.CV420YpCbCr8BiPlanarFullRange))
                            {
                                Debug($"skipping format with reduced color space: {format}");
                                considerThisFormat = false;
                            }

                            if (considerThisFormat && (foundVideoFormat?.HighestPhotoQualitySupported ?? false) && !format.HighestPhotoQualitySupported)
                            {
                                Debug($"skipping format without highest quality photo: {format}");
                                considerThisFormat = false;
                            }
                            var iosVer =  int.Parse(DeviceInfo.Current.VersionString.Split('.')[0]);
                            if (considerThisFormat && iosVer > 15)
                            {
                                if (considerThisFormat && foundVideoFormat.HighPhotoQualitySupported && !format.HighPhotoQualitySupported)
                                {
                                    Debug($"skipping format without high quality photo:{format}");
                                    considerThisFormat = false;
                                }
                            }
                        }

                        // fallthrough

                        if (considerThisFormat || (foundMaxResolution < desiredVideoResolution) && (dimHeight > foundMaxResolution))
                        {
                            // We haven't found the resolution we need yet, or this is better than the old one.
                            Debug($"found potential video format: {format} {frr}");
                            foundVideoFormat = format;
                            foundMaxFramerate = (float)frr.MaxFrameRate;
                            foundMaxResolution = (int)dimHeight;
                            foundFoV = format.VideoFieldOfView;
                            foundStillResolution = stillResolution;
                            selectedVideoResolution = (int)dimHeight;
                        }
                        else
                        {
                            Debug($"not considering:{format} {frr}");
                        }
                    }
                }
            }
        }
        if(foundVideoFormat != null) {
            Debug($"selected video format: {foundVideoFormat}");
            selectedVideoFramerate = Math.Min(foundMaxFramerate, desiredVideoFramerate);
            Debug($"selected video frame rate: ({selectedVideoFramerate}) fps");
            var frameDuration = new CMTime(value: 1, timescale: (int)selectedVideoFramerate);

            videoCaptureDevice.LockForConfiguration(out NSError error);
            if (error == null)
            {
                videoCaptureDevice.ActiveFormat = foundVideoFormat!;
                videoCaptureDevice.ActiveVideoMinFrameDuration = frameDuration;
                videoCaptureDevice.ActiveVideoMaxFrameDuration = frameDuration;
                videoCaptureDevice.UnlockForConfiguration();
            }
            else
            {
                Debug($"failed to set video format:{error}");
            }
        }
        var supportedVideoCodecs = (otherRecordingParameters?.SupportedVideoCodecs?.Any(x => x.Any()) ?? false) ? (otherRecordingParameters?.SupportedVideoCodecs) : new string[] { _strH264 };

        Debug($"want one of these codecs: {string.Join(", ", supportedVideoCodecs)}");
        var availableVideoCodecTypes = videoOutput.AvailableVideoCodecTypes.Select(x => x.ToString());
        Debug($"have these codecs available: {string.Join(", ", availableVideoCodecTypes)}");

        var done = false;
        foreach (var  supportedVideoCodec in supportedVideoCodecs)
        {
            var supportedAvVideoCodecType = CodecFromString(supportedVideoCodec);

            if (supportedAvVideoCodecType != null)
            {
                var strAvVideoCodecType = AVFoundation.AVVideoCodecTypeExtensions.GetConstant((AVVideoCodecType)supportedAvVideoCodecType).ToString();
                var strVideoCodecTypeFound = availableVideoCodecTypes.SingleOrDefault(x => (x  == strAvVideoCodecType));
                if (strVideoCodecTypeFound?.Any() ?? false)
                {
                    Debug($"selecting codec: {strVideoCodecTypeFound}");

                    var newSettings = new NSDictionary(
                        AVVideo.CodecKey, new NSString(strVideoCodecTypeFound)
                    );

                    videoOutput.SetOutputSettings(newSettings, videoOutputConnection);
                    done = true;
                    break;
                }
            }
        }

        if (!done) {
            Debug("failed to locate any desired video codec");
            retVal = false;
        }
        return retVal;
    }

    public bool SetFocus(Microsoft.Maui.Graphics.Rect rect)
    {
        bool retval = false;
        if (cameraView.Camera != null && captureDevice != null && captureDevice.IsFocusModeSupported(AVCaptureFocusMode.AutoFocus))
        {
            captureDevice.LockForConfiguration(out NSError error);
            if (error == null)
            {
                //HO 0.5 0.5 is what it is originally
                var focusPointOfInterest = new CGPoint(0.5f, 0.5f);
                if (rect != Rect.Zero)
                {
                    focusPointOfInterest = PreviewLayer.CaptureDevicePointOfInterestForPoint(new CGPoint(rect.Center.X, rect.Center.Y));
                }
                captureDevice.FocusPointOfInterest = focusPointOfInterest;

                _logger_LogTrace?.Invoke($"Converting from view coordinates too sensor coordinates");
                _logger_LogTrace?.Invoke($"CameraView Size: {new Microsoft.Maui.Graphics.Size(cameraView.Width, cameraView.Height)}");
                _logger_LogTrace?.Invoke($"CameraView Focus Point: {rect.Center} Sensor Relative Focus Rect: {focusPointOfInterest}  ");

                var continousAutoFocusSupported = captureDevice.IsFocusModeSupported(AVCaptureFocusMode.ContinuousAutoFocus);
                _logger_LogTrace?.Invoke($"{nameof(continousAutoFocusSupported)}: {continousAutoFocusSupported}");

                if (continousAutoFocusSupported)
                {
                    captureDevice.FocusMode = AVCaptureFocusMode.AutoFocus;
                    captureDevice.FocusMode = AVCaptureFocusMode.ContinuousAutoFocus;
                }
                else
                {
                    captureDevice.FocusMode = AVCaptureFocusMode.ContinuousAutoFocus;
                    captureDevice.FocusMode = AVCaptureFocusMode.AutoFocus;
                }

                captureDevice.UnlockForConfiguration();
                retval = true;
            }
        }
        return retval;
    }
}


#endif
