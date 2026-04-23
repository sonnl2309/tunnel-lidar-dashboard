#include "TunnelRegistration.h"
#include <cmath>
#include <algorithm>
#include <iostream>
#include <map> 

TunnelRegistration::TunnelRegistration() {}
TunnelRegistration::~TunnelRegistration() {}

// ============================================================
// TỰ ĐỘNG XÁC ĐỊNH PHẠM VI GƯƠNG HẦM (BASED ON HISTOGRAM PEAK)
// ============================================================
void TunnelRegistration::detectTunnelFaceRange(CCCoreLib::GenericIndexedCloudPersist* cloud, 
                                              float& outYMin, float& outYMax, 
                                              float sliceThickness) {
    if (!cloud || cloud->size() == 0) return;

    CCVector3 bbMin, bbMax;
    cloud->getBoundingBox(bbMin, bbMax);

    std::map<int, unsigned> histogram;
    for (unsigned i = 0; i < cloud->size(); ++i) {
        const CCVector3* P = cloud->getPoint(i);
        int sliceIdx = static_cast<int>((P->y - bbMin.y) / sliceThickness);
        histogram[sliceIdx]++;
    }

    int peakSliceIdx = 0;
    unsigned maxPoints = 0;
    for (auto const& [idx, count] : histogram) {
        if (count > maxPoints) {
            maxPoints = count;
            peakSliceIdx = idx;
        }
    }

    float peakY = bbMin.y + (peakSliceIdx * sliceThickness) + (sliceThickness / 2.0f);
    
    // Lấy khoảng đệm hẹp dọc trục Y (ví dụ 1 mét)
    outYMin = peakY - 500.0f; 
    outYMax = peakY + 500.0f; 
    
    outYMin = std::max(outYMin, bbMin.y);
    outYMax = std::min(outYMax, bbMax.y);
}

// ============================================================
// BƯỚC 2: CÔ LẬP VÙNG TRUNG TÂM GƯƠNG HẦM (UPGRADED SEGMENTATION)
// Logic mới: Lọc theo 3 trục để lấy vùng đa giác ở giữa như image_dd30c3.jpg
// ============================================================
CCCoreLib::ReferenceCloud* TunnelRegistration::isolateTunnelFace(CCCoreLib::GenericIndexedCloudPersist* cloud, 
                                                                float yMin, float yMax) {
    if (!cloud) return nullptr;

    CCVector3 bbMin, bbMax;
    cloud->getBoundingBox(bbMin, bbMax);

    // Xác định kích thước gương hầm
    float width = bbMax.x - bbMin.x;
    float height = bbMax.z - bbMin.z;

    // Định nghĩa vùng "Core" (vùng lõi) như trong ảnh image_dd30c3.jpg
    // Lấy 60% ở giữa theo phương ngang (X) và 60% ở phần trên theo phương đứng (Z)
    // Loại bỏ 20% mỗi bên vách và 30% phần nền (thường chứa đất đá vụn)
    float xMinCore = bbMin.x + width * 0.20f;
    float xMaxCore = bbMax.x - width * 0.20f;
    float zMinCore = bbMin.z + height * 0.30f; // Loại bỏ nền
    float zMaxCore = bbMax.z - height * 0.05f; // Loại bỏ trần sát mép

    CCCoreLib::ReferenceCloud* faceCloud = new CCCoreLib::ReferenceCloud(cloud);

    for (unsigned i = 0; i < cloud->size(); ++i) {
        const CCVector3* P = cloud->getPoint(i);
        
        // Kiểm tra điểm có nằm trong khối hộp trung tâm không
        bool inY = (P->y >= yMin && P->y <= yMax);
        bool inX = (P->x >= xMinCore && P->x <= xMaxCore);
        bool inZ = (P->z >= zMinCore && P->z <= zMaxCore);

        if (inY && inX && inZ) {
            faceCloud->addPointIndex(i);
        }
    }

    if (faceCloud->size() == 0) {
        delete faceCloud;
        return nullptr;
    }

    std::cout << "[Step 2] Isolate core region successful. Points: " << faceCloud->size() << std::endl;
    return faceCloud;
}

// ============================================================
// BƯỚC 3: KHỚP NỐI TINH (ICP REGISTRATION)
// ============================================================
bool TunnelRegistration::runAutoICP(CCCoreLib::GenericIndexedCloudPersist* source, 
                                   CCCoreLib::GenericIndexedCloudPersist* reference,
                                   double overlap) {
    if (!source || !reference) return false;

    CCCoreLib::RegistrationTools::Parameters params;
    params.convType = CCCoreLib::RegistrationTools::MAX_ERROR_CONVERGENCE;
    params.errorConvergenceThreshold = 1.0e-6; 
    params.maxIterationCount = 50;
    params.randomSamplingLimit = 50000; 
    params.finalOverlapRatio = overlap; 

    CCCoreLib::RegistrationTools::ScaledTransformation res;
    double finalError = 0;
    unsigned finalPointCount = 0;

    bool success = CCCoreLib::RegistrationTools::Register(
        source, 
        nullptr, 
        reference, 
        params, 
        res, 
        finalError, 
        finalPointCount
    );

    if (success) {
        source->applyRigidTransformation(res.R, res.T);
    }

    return success;
}

// ============================================================
// BƯỚC 4: ĐÁNH GIÁ SAI SỐ (EVALUATION MODULE)
// ============================================================
double TunnelRegistration::computeTrimmedRMSE(CCCoreLib::GenericIndexedCloudPersist* source, 
                                            CCCoreLib::GenericIndexedCloudPersist* reference,
                                            unsigned& outInlierCount) {
    CCCoreLib::DistanceComputationTools::Cloud2CloudDistanceComputationParams distParams;
    distParams.octreeLevel = 10;

    if (CCCoreLib::DistanceComputationTools::computeCloud2CloudDistance(source, reference, distParams) < 0)
        return -1.0;

    double sumSqDiff = 0;
    outInlierCount = 0;
    const double threshold = 100.0; 

    for (unsigned i = 0; i < source->size(); ++i) {
        ScalarType dist = source->getPointScalarValue(i);
        if (dist <= threshold) {
            sumSqDiff += (dist * dist);
            outInlierCount++;
        }
    }

    if (outInlierCount == 0) return -1.0;
    return std::sqrt(sumSqDiff / outInlierCount);
}

EvaluationResult TunnelRegistration::evaluate(CCCoreLib::GenericIndexedCloudPersist* source, 
                                             CCCoreLib::GenericIndexedCloudPersist* reference) {
    EvaluationResult res;
    unsigned nPoints = 0;
    
    res.trimmedRMSE = computeTrimmedRMSE(source, reference, nPoints);
    res.inlierCount = nPoints;

    double fitnessScore = (double)nPoints / source->size();
    double rmse_cm = res.trimmedRMSE / 10.0; 

    if (fitnessScore < 0.5 || rmse_cm > 5.0) {
        res.verdict = "FAIL (Vượt ngưỡng an toàn)";
        res.sanityCheck = false;
    } else if (rmse_cm < 2.5) {
        res.verdict = "Tuyệt hảo (Mức cảm biến vật lý)";
        res.sanityCheck = true;
    } else if (rmse_cm <= 4.5) {
        res.verdict = "PASS (Đạt tiêu chuẩn bề mặt đá thô)";
        res.sanityCheck = true;
    } else {
        res.verdict = "FAIL (Vượt ngưỡng 4.5cm)";
        res.sanityCheck = false;
    }

    return res;
}