/*********************************************************************
Copyright (c) 2018
Audi Autonomous Driving Cup. Team FAUtonomous. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: ?This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.?
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

TODO: write a short description of your filter!!
This filter is our template filter for aadc2018.
Use this filter for creating your own filters.
Keep in mind to change "cLineFilter" to your filtername.

**********************************************************************/

#include "stdafx.h"
#include "LineFilter.h"

// This will define the filter and expose it via plugin class factory.
// Class cLineFilter will be embedded to the filter and called by
// data trigger of "speed" by default.
//ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(
//    CID_LINE_FILTER_FILTER,		// references to header file
//    "cLineFilter",              // label
//    cLineFilter,                // class
//    adtf::filter::pin_trigger({"input"}));	// set trigger pin

#include "ImageProcessingUtils.h"
#include "CVMath.h"
//
#include <opencv2/opencv.hpp>
#include <iostream>


const tFloat32 cLineFilter::SIZE_X = 1.5;
const tFloat32 cLineFilter::SIZE_Y = 1.5;

cLineFilter::cLineFilter()
{
        //Some default parameters
        this->_near = 0.7;
        this->_far = 2.2;

        this->_fovHorizontal = 58;
        this->_fovVertical = 45;

        this->_pitch = 0;
        this->_cameraOffsetX = -0.015;
        this->_cameraOffsetZ = 0.22;

        this->_cameraWidth = 1280;
        this->_cameraHeight = 720;

        _adaptiveThresholdConstant = 21;
        _enableAdaptiveThresholding = true;

        _maskWorkRegion = 0;

        _enableMedianFiltering = false;
        _isInitialized = false;
}

cLineFilter::~cLineFilter()
{
}

void cLineFilter::Init(tFloat32 near, tFloat32 fovHorizontal, tFloat32 fovVertical, tFloat32 pitch, tFloat32 cameraPositionY,
                       tFloat32 cameraPositionZ, tFloat32 adaptiveThresholdConstant, tUInt32 adaptiveThresholdWidth,
                       tUInt32 cameraWidth, tUInt32 cameraHeight)
{
        this->_near = near;
        this->_far  = near + SIZE_Y;

        this->_fovHorizontal = fovHorizontal;
        this->_fovVertical   = fovVertical;

        this->_pitch = pitch;
        this->_cameraOffsetX = - cameraPositionY;
        this->_cameraOffsetZ = cameraPositionZ;

        this->_cameraWidth  = cameraWidth;
        this->_cameraHeight = cameraHeight;

        this->_adaptiveThresholdConstant  = adaptiveThresholdConstant;
        this->_adaptiveThresholdWidth     = adaptiveThresholdWidth;
        this->_enableAdaptiveThresholding = true;

        _maskWorkRegion = 0;

        _gray        = Mat(Size(BINARY_IMAGE_WIDTH, BINARY_IMAGE_HEIGHT), CV_8UC1, Scalar(0));
        _validPixels = Mat(Size(BINARY_IMAGE_WIDTH, BINARY_IMAGE_HEIGHT), CV_8UC1, Scalar(0));
        _warp        = Mat(Size(BINARY_IMAGE_WIDTH, BINARY_IMAGE_HEIGHT), CV_8UC3, Scalar(0, 0, 0));
        _affineMask  = Mat(Size(BINARY_IMAGE_WIDTH, BINARY_IMAGE_HEIGHT), CV_8UC1, Scalar(0));

        _isInitialized = true;

        CalculatePerspectiveTransformationMatrix();
}

void cLineFilter::EnableAdaptiveThresholding(tBool enable)
{
        this->_enableAdaptiveThresholding = enable;
}

void cLineFilter::EnableMedianFiltering(tBool enable)
{
        this->_enableMedianFiltering = enable;
}

void cLineFilter::SetNearFar(tFloat32 near)
{
        this->_near = near;
        this->_far  = near + SIZE_Y;

        CalculatePerspectiveTransformationMatrix();
}

void cLineFilter::SetFOV(tFloat32 fovHorizontal, tFloat32 fovVertical)
{
        this->_fovHorizontal = fovHorizontal;
        this->_fovVertical   = fovVertical;

        CalculatePerspectiveTransformationMatrix();
}

void cLineFilter::SetPitch(tFloat32 pitch)
{
        this->_pitch = pitch;

        CalculatePerspectiveTransformationMatrix();
}

void cLineFilter::SetCameraPosition(tFloat32 cameraPositionX, tFloat32 cameraPositionZ)
{
        this->_cameraOffsetX = cameraPositionX;
        this->_cameraOffsetZ = cameraPositionZ;

        CalculatePerspectiveTransformationMatrix();
}

void cLineFilter::SetCameraSize(tUInt32 cameraWidth, tUInt32 cameraHeight)
{
        this->_cameraWidth  = cameraWidth;
        this->_cameraHeight = cameraHeight;

        CalculatePerspectiveTransformationMatrix();
}

void cLineFilter::SetAdaptiveThresholdConstant(tFloat32 adaptiveThresholdConstant)
{
        this->_adaptiveThresholdConstant = adaptiveThresholdConstant;
}

void cLineFilter::SetValidPixels(cv::Mat &mask, Size imageSize, tBool enable)
{
        if(enable)
        {
                tUInt32 cropWidth = 20;
                tUInt32 maskWidth = mask.cols-cropWidth;
                tUInt32 outHeight = (imageSize.height >> 1) - _maskWorkRegion;
                //tUInt32 outWidth = 0.95 * maskWidth;//measured 0.95
                tUInt32 outWidth = (imageSize.width >> 1);

                mask(Rect(0, _maskWorkRegion, mask.cols, mask.rows - _maskWorkRegion)).copyTo(_validPixels);

                if(!_lastMask.empty())
                {
                        bitwise_or(_validPixels, _lastMask, _buffer);
                        if(!_lastlastMask.empty())
                        {
                                bitwise_or(_buffer, _lastlastMask, _buffer);
                        }

                        resize(_buffer(Rect(0, 0, maskWidth, _buffer.rows)), _buffer, Size(outWidth, outHeight));

                        _lastMask.copyTo(_lastlastMask);
                }
                else
                {
                        resize(_validPixels(Rect(0, 0, maskWidth, _validPixels.rows)), _buffer, Size(outWidth, outHeight));
                }

                _validPixels.copyTo(_lastMask);

                if(outWidth != (imageSize.width >> 1))
                {
                        copyMakeBorder(_buffer, _buffer, 0, 0, 0, (imageSize.width >> 1) - outWidth, BORDER_REPLICATE);
                }

                if(1)
                {
                        //Mat element = getStructuringElement(MORPH_RECT, Size( 12, 2));
                        //cv::erode(buffer, validPixels, element, Point(-1,-1), 1);

                        blur(_buffer, _validPixels, Size(1, 1));
                }
                else
                {
                        _buffer.copyTo(_validPixels);
                }

                if(!_lastValidPixels.empty())
                {
                        multiply(_validPixels, _lastValidPixels, _validPixels, 1.0/255);
                }

                if(1)
                {
                        Mat element = getStructuringElement( MORPH_RECT, Size( 4, 3));
                        cv::dilate(_validPixels, _lastValidPixels, element, Point(-1,-1), 1);
                }

                copyMakeBorder(_validPixels, _validPixels, _maskWorkRegion, 0, 0, 0, BORDER_CONSTANT);
                resize(_validPixels, _validPixels, imageSize);
        }
        else
        {
                _validPixels = Mat::ones(imageSize, CV_8UC1) * 255;
        }
}

tResult cLineFilter::GetTopDownView(Mat &in, Mat &roi, Mat& out, tBool use_CV_rgb2gray)
{
    CalculatePerspectiveTransformationMatrix();
    if(!_isInitialized)
    {
            RETURN_AND_LOG_ERROR((tResult)0);
    }

    if(use_CV_rgb2gray)
    {
            cvtColor(in,in,COLOR_RGB2GRAY);
    }
    else
    {
            cvtColor(in,in,COLOR_BGR2GRAY);
    }
    imwrite("/tmp/1in.jpg", in);
    imwrite("/tmp/2valid.jpg", roi);
    imwrite("/tmp/3buffernachmul.jpg", _buffer);
    //multiply(in, _validPixels, _buffer, 1.0/255);
    multiply(in, roi, _buffer, 1.0/255);

    if(_enableAdaptiveThresholding)
    {
        warpPerspective(_buffer, _warp, _topDownMatrix, _warp.size(), INTER_LINEAR, BORDER_REPLICATE);
        adaptiveThreshold(_warp, _gray, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY, _adaptiveThresholdWidth, _adaptiveThresholdConstant);
        _gray.copyTo(out, _affineMask);
    }
    else
    {
        warpPerspective(_buffer, _warp, _topDownMatrix, _warp.size(), INTER_LINEAR, BORDER_REPLICATE);
        imwrite("/tmp/4warp.jpg", _warp);
        //warp = warp - 32;
        //out.setTo(128);
        _warp.copyTo(out, _affineMask);
        imwrite("/tmp/5out.jpg", _warp);
    }

    if(_enableMedianFiltering)
    {
            medianBlur(out, out, 3);
            imwrite("/tmp/6ouEMFt.jpg", out);
    }

    RETURN_NOERROR;
}

Point2f cLineFilter::GetTopLeftWorldCoordinates()
{
        return Point2f(_far, SIZE_X * 0.5);
}

void cLineFilter::CalculatePerspectiveTransformationMatrix()
{
        Point2f sourcePoints[4] = { Point2f(0, 0), Point2f(0, 0), Point2f(0, 0), Point2f(0, 0) };
        CalculateSourcePoints(sourcePoints);

        _maskWorkRegion = (sourcePoints[0].y) / 2;
        if(_maskWorkRegion < 0)
        {
                _maskWorkRegion = 0;
        }

        Point2f destPoints[4] = { Point2f(100, 0), Point2f(200, 0), Point2f(200, 300), Point2f(100, 300) };
        _topDownMatrix = getPerspectiveTransform(sourcePoints, destPoints);

        try
        {
                Mat white = Mat(this->_cameraHeight, this->_cameraWidth, CV_8UC1, Scalar(255));
                warpPerspective(white, _affineMask, _topDownMatrix, _affineMask.size(), INTER_LINEAR, BORDER_CONSTANT, 0);

        }
        catch (cv::Exception &e)
        {
                std::cout << e.msg << std::endl;
        }
}

void cLineFilter::CalculateSourcePoints(cv::Point2f* sourcePoints)
{
        if(!_isInitialized)
        {
                return;
        }

        tFloat32 vertical   = tan(_fovVertical   / 180 * CV_PI / 2);
        tFloat32 horizontal = tan(_fovHorizontal / 180 * CV_PI / 2);

        Point3f vectorFarLeft (-0.25 - _cameraOffsetX, _cameraOffsetZ, _far);
        Point3f vectorFarRight (0.25 - _cameraOffsetX, _cameraOffsetZ, _far);
        Point3f vectorNearRight(0.25 - _cameraOffsetX, _cameraOffsetZ, _near);
        Point3f vectorNearLeft(-0.25 - _cameraOffsetX, _cameraOffsetZ, _near);

        std::vector<Point2f> vec;
        vec.push_back(Point2f(vectorNearRight.y, vectorNearRight.z));
        vec.push_back(Point2f(vectorFarRight.y , vectorFarRight.z));

        //clockwise rotation with pitch angle
        Mat r = getRotationMatrix2D(Point2f(0, 0), _pitch, 1.0);
        transform(vec, vec, r);

        Point2f vectorNear = vec.at(0);
        Point2f vectorFar  = vec.at(1);
        tFloat32 nearY = (_cameraHeight / 2) + vectorNear.x / (vectorNear.y * vertical) * (_cameraHeight / 2);
        tFloat32 farY  = (_cameraHeight / 2) + vectorFar.x  / (vectorFar.y  * vertical) * (_cameraHeight / 2);


        // camera_width       vectorFarLeft.x * (camera_width / 2)
        // ------------  +   -------------------------------------
        //       2                       (far * horizontal)


        tFloat32 farLeftX   = (_cameraWidth / 2) + vectorFarLeft.x   / (vectorFar.y * horizontal)  * (_cameraWidth / 2);
        tFloat32 farRightX  = (_cameraWidth / 2) + vectorFarRight.x  / (vectorFar.y * horizontal)  * (_cameraWidth / 2);
        tFloat32 nearRightX = (_cameraWidth / 2) + vectorNearRight.x / (vectorNear.y * horizontal) * (_cameraWidth / 2);
        tFloat32 nearLeftX  = (_cameraWidth / 2) + vectorNearLeft.x  / (vectorNear.y * horizontal) * (_cameraWidth / 2);

        sourcePoints[0] = Point2f(farLeftX  , farY);    // Top left
        sourcePoints[1] = Point2f(farRightX , farY);   // Top right
        sourcePoints[2] = Point2f(nearRightX, nearY); // Bottom right
        sourcePoints[3] = Point2f(nearLeftX , nearY);  // Bottom left
/*
        for (tUInt32 i = 0; i < 4; ++i)
        {
                LOG_INFO(cString::Format("SourcePoint[%d] x, y: %f, %f",i, sourcePoints[i].x, sourcePoints[i].y ));
        }*/
}
