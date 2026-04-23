#ifndef TUNNEL_REGISTRATION_H
#define TUNNEL_REGISTRATION_H

#include <CCCoreLib.h>
#include <PointCloud.h>
#include <RegistrationTools.h>
#include <DistanceComputationTools.h>
#include <vector>
#include <string>

// Cấu trúc chứa kết quả đánh giá cuối cùng
struct EvaluationResult {
    double trimmedRMSE;     // Sai số bình phương trung bình phần lõi
    unsigned inlierCount;   // Số lượng điểm lọt lưới (N)
    std::string verdict;    // Kết luận: Tuyệt hảo, Đạt, hoặc FAIL
    bool sanityCheck;       // Kiểm tra tính logic của ma trận
};

class TunnelRegistration {
public:
    TunnelRegistration();
    ~TunnelRegistration();

    // Bước 1: Nạp dữ liệu (Giả định dữ liệu đã được nạp vào đối tượng PointCloud)
    // Trong thực tế, bạn cần một module loader để chuyển CSV/BIN thành ccPointCloud
    
    // Bước 2 & 3: Thực hiện quy trình ICP tự động
    bool runAutoICP(CCCoreLib::GenericIndexedCloudPersist* source, 
                    CCCoreLib::GenericIndexedCloudPersist* reference,
                    double overlap = 0.8);

    // Bước 4: Đánh giá sai số (Evaluator)
    EvaluationResult evaluate(CCCoreLib::GenericIndexedCloudPersist* source, 
                              CCCoreLib::GenericIndexedCloudPersist* reference);

private:
    // Hỗ trợ tính toán RMSE trên tập hợp điểm N (Trimmed RMSE)
    double computeTrimmedRMSE(CCCoreLib::GenericIndexedCloudPersist* source, 
                              CCCoreLib::GenericIndexedCloudPersist* reference,
                              unsigned& outInlierCount);
};

#endif