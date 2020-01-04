// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#include <iostream>
#include <vector>
#include <k4a/k4a.h>
#include <k4arecord/playback.h>
#include <string>
#include "transformation_helpers.h"
#include "turbojpeg.h"

#include "k4adepthpixelcolorizer.h"
#include "k4apixel.h"
#include "k4astaticimageproperties.h"
#include "texture.h"
#include "viewerwindow.h"
#include <unistd.h>

using namespace viewer;
using namespace std; 
void ColorizeDepthImage(const k4a::image &depthImage,
                        DepthPixelVisualizationFunction visualizationFn,
                        std::pair<uint16_t, uint16_t> expectedValueRange,
                        std::vector<BgraPixel> *buffer);
uint16_t a[720][1280];
uint16_t* dataread(){
        return (uint16_t*)a;
}

uint8_t b[720][1280][4];
uint8_t* dataread_color(){
        return (uint8_t*)b;
}

uint16_t c[512][512];
uint16_t* dataread_depth(){
        return (uint16_t*)c;
}
uint8_t d[512][512][4];
uint8_t* dataread_color_to_depth(){
        return (uint8_t*)d;
}
k4a_device_t device = NULL;
int main(){
 return -1;
}
k4a::device dev = k4a::device::open(K4A_DEVICE_DEFAULT);
k4a::calibration calibration =dev.get_calibration(K4A_DEPTH_MODE_WFOV_2X2BINNED,K4A_COLOR_RESOLUTION_720P );
k4a::image depthImage_temp;
float xyz[3];
float* convert_2d_to_3d_(int x, int y){
    k4a_float2_t p;
    k4a_float3_t ray;
    float depth = c[int(y-1)][int(x-1)];
    p.xy.x = x;
    p.xy.y = y;
    calibration.convert_2d_to_3d(p,depth,K4A_CALIBRATION_TYPE_DEPTH,K4A_CALIBRATION_TYPE_DEPTH,&ray);
    xyz[0] = ray.xyz.x;
    xyz[1] = ray.xyz.y;
    xyz[2] = ray.xyz.z;
    return xyz;
}
float xy[2];
float* convert_2d_to_2d_(int x, int y){
    k4a_float2_t p;
    k4a_float2_t ray;
    p.xy.x = x;
    p.xy.y = y;
    calibration.convert_color_2d_to_depth_2d(p,depthImage_temp,&ray);
    xy[0] = ray.xy.x;
    xy[1] = ray.xy.y;

    return xy;
}

int start(){

    

    //const int32_t TIMEOUT_IN_MS = 1000;
   // k4a_transformation_t transformation = NULL;
    std::string file_name = "";
    std::vector<BgraPixel> depthTextureBuffer;
    std::vector<BgraPixel> transform_depthTextureBuffer;
    
    k4a::transformation kTransformation(calibration);
    k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;

    config.camera_fps = K4A_FRAMES_PER_SECOND_30;
    config.depth_mode = K4A_DEPTH_MODE_WFOV_2X2BINNED;
    config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
    config.color_resolution = K4A_COLOR_RESOLUTION_720P;


	// This means that we'll only get captures that have both color and
	// depth images, so we don't need to check if the capture contains
	// a particular type of image.
	//
    config.synchronized_images_only = true;
    dev.start_cameras(&config);
    while(1){
         // Get a capture
         k4a::capture capture;
         
         if (dev.get_capture(&capture, std::chrono::milliseconds(0)))
        {
            const k4a::image depthImage = capture.get_depth_image();

            const k4a::image colorImage = capture.get_color_image();
            const k4a::image transformedImage = kTransformation.depth_image_to_color_camera(depthImage);
            const k4a::image ctransformedImage = kTransformation.color_image_to_depth_camera(depthImage,colorImage);
            //const uint16_t *transformed_depthData = reinterpret_cast<const uint16_t *>(transformedImage.get_buffer());
            const uint8_t *colorData = (colorImage.get_buffer());
            const uint16_t *transformed_depthData = reinterpret_cast<const uint16_t *>(transformedImage.get_buffer());
            const uint16_t *depthData = reinterpret_cast<const uint16_t *>(depthImage.get_buffer());
            depthImage_temp = depthImage;
            const uint8_t *transformed_colorData = (ctransformedImage.get_buffer());
            int n1=0;
            int n2=0;
            int n3=0;
            int n4=0;
	    for (int h = 0; h < 720; ++h)
	    {
		for (int w = 0; w < 1280; ++w)
		{
			a[h][w] = transformed_depthData[n1++];
			b[h][w][0] = colorData[n2++];
			b[h][w][1] = colorData[n2++];
			b[h][w][2] = colorData[n2++];
			b[h][w][3] = colorData[n2++];
		}
	    }
	    for (int h = 0; h < 512; ++h)
	    {
		for (int w = 0; w < 512; ++w)
		{
	              c[h][w] = depthData[n3++];
		      d[h][w][0] = transformed_colorData[n4++];
		      d[h][w][1] = transformed_colorData[n4++];
		      d[h][w][2] = transformed_colorData[n4++];
	              d[h][w][3] = transformed_colorData[n4++];
		}
	    }

         }

         

    }
}
	









//-----------------------------------------------------------------------------------------


int maink()
{
    try
    { 
        // Check for devices
        //
        const uint32_t deviceCount = k4a::device::get_installed_count();
        if (deviceCount == 0)
        {
            throw std::runtime_error("No Azure Kinect devices detected!");
        }

        // Start the device
        //
        k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
        
        config.camera_fps = K4A_FRAMES_PER_SECOND_30;
        config.depth_mode = K4A_DEPTH_MODE_WFOV_2X2BINNED;
        config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
        config.color_resolution = K4A_COLOR_RESOLUTION_720P;


        // This means that we'll only get captures that have both color and
        // depth images, so we don't need to check if the capture contains
        // a particular type of image.
        //
        config.synchronized_images_only = true;

        std::cout << "Started opening K4A device..." << std::endl;




	uint32_t device_count = k4a_device_get_installed_count();

	if (device_count == 0)
	{
		printf("No K4A devices found\n");
		return 0;
	}
        printf("Device COUNT : %d \n\n",device_count);

        k4a::device dev = k4a::device::open(K4A_DEVICE_DEFAULT);

        dev.start_cameras(&config);


        std::cout << "Finished opening K4A device." << std::endl;
        // Create the viewer window.
        //
        ViewerWindow &window = ViewerWindow::Instance();
        window.Initialize("Simple Azure Kinect Viewer", 1440, 900);

        // Textures we can give to OpenGL / the viewer window to render.
        //
        Texture depthTexture = window.CreateTexture(GetDepthDimensions(config.depth_mode));
        Texture colorTexture = window.CreateTexture(GetColorDimensions(config.color_resolution));

        // A buffer containing a BGRA color representation of the depth image.
        // This is what we'll end up giving to depthTexture as an image source.
        // We don't need a similar buffer for the color image because the color
        // image already comes to us in BGRA32 format.
        //
        std::vector<BgraPixel> depthTextureBuffer;

        // viewer.BeginFrame() will start returning false when the user closes the window.
        //
        while (window.BeginFrame())
        {
            // Poll the device for new image data.
            //
            // We set the timeout to 0 so we don't block if there isn't an available frame.
            //
            // This works here because we're doing the work on the same thread that we're
            // using for the UI, and the ViewerWindow class caps the framerate at the
            // display's refresh rate (the EndFrame() call blocks on the display's sync).
            //
            // If we don't have new image data, we'll just reuse the textures we generated
            // from the last time we got a capture.
            //
            k4a::capture capture;
            
            if (dev.get_capture(&capture, std::chrono::milliseconds(0)))
            {
                const k4a::image depthImage = capture.get_depth_image();
                const k4a::image colorImage = capture.get_color_image();

                // If we hadn't set synchronized_images_only=true above, we'd need to do
                // if (depthImage) { /* stuff */ } else { /* ignore the image */ } here.
                //
                // Depth data is in the form of uint16_t's representing the distance in
                // millimeters of the pixel from the camera, so we need to convert it to
                // a BGRA image before we can upload and show it.
                //
                ColorizeDepthImage(depthImage,
                                   K4ADepthPixelColorizer::ColorizeBlueToRed,
                                   GetDepthModeRange(config.depth_mode),
                                   &depthTextureBuffer);
                depthTexture.Update(&depthTextureBuffer[0]);

                // Since we're using BGRA32 mode, we can just upload the color image directly.
                // If you want to use one of the other modes, you have to do the conversion
                // yourself.
                //
                colorTexture.Update(reinterpret_cast<const BgraPixel *>(colorImage.get_buffer()));
            }

            // Show the windows
            //
            const ImVec2 windowSize(window.GetWidth() / 2.f, static_cast<float>(window.GetHeight()));

            window.ShowTexture("Depth", depthTexture, ImVec2(0.f, 0.f), windowSize);
            window.ShowTexture("Color", colorTexture, ImVec2(window.GetWidth() / 2.f, 0.f), windowSize);

            // This will tell ImGui/OpenGL to render the frame, and will block until the next vsync.
            //
            window.EndFrame();
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << std::endl;
        std::cerr << "Press [enter] to exit." << std::endl;
        std::cin.get();
        return 1;
    }
    return 0;
}

// Given a depth image, output a BGRA-formatted color image into buffer, using
// expectedValueRange to define what min/max values the depth image should have.
// Low values are blue, high values are red.
//
void ColorizeDepthImage(const k4a::image &depthImage,
                        DepthPixelVisualizationFunction visualizationFn,
                        std::pair<uint16_t, uint16_t> expectedValueRange,
                        std::vector<BgraPixel> *buffer)
{
    // This function assumes that the image is made of depth pixels (i.e. uint16_t's),
    // which is only true for IR/depth images.
    //
    const k4a_image_format_t imageFormat = depthImage.get_format();
    if (imageFormat != K4A_IMAGE_FORMAT_DEPTH16 && imageFormat != K4A_IMAGE_FORMAT_IR16)

    {
        throw std::logic_error("Attempted to colorize a non-depth image!");
    }

    const int width = depthImage.get_width_pixels();
    const int height = depthImage.get_height_pixels();

    buffer->resize(static_cast<size_t>(width * height));

    const uint16_t *depthData = reinterpret_cast<const uint16_t *>(depthImage.get_buffer());
    for (int h = 0; h < height; ++h)
    {
        for (int w = 0; w < width; ++w)
        {
            const size_t currentPixel = static_cast<size_t>(h * width + w);
            (*buffer)[currentPixel] = visualizationFn(depthData[currentPixel],
                                                      expectedValueRange.first,
                                                      expectedValueRange.second);
        }
    }
}
int end(){
     k4a_device_close(device);
     exit(0);

    return -1;
}
extern "C" {
	int Foo_start(){start(); return 0;}
        uint16_t*  Foo_dataread(){return dataread();}
	int Foo_end(){end(); return 0;}
        uint8_t*  Foo_dataread_color(){return dataread_color();}
        uint16_t*  Foo_dataread_depth(){return dataread_depth();}
        float*   Foo_convert_2d_3d(int x,int y){return convert_2d_to_3d_(x,y);}
        float*   Foo_convert_2d_2d(int x,int y){return convert_2d_to_2d_(x,y);}
        uint8_t*  Foo_dataread_color_to_depth(){return dataread_color_to_depth();}
}
