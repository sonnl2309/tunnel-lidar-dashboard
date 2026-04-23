#include "TunnelRegistration.h"
#include <iostream>

// Hàm giả định để minh họa luồng tích hợp
int main() {
    // 1. Khởi tạo đối tượng xử lý
    TunnelRegistration pipeline;

    // Giả sử cloudSource và cloudReference đã được load từ CSV vào ccPointCloud (thư viện CC)
    // ccPointCloud* cloudSource = ...
    // ccPointCloud* cloudReference = ...

    std::cout << "Starting Automated Tunnel Registration Pipeline..." << std::endl;

    // 2. Thực hiện ICP tự động (Bước 3 trong Workflow)
    // bool success = pipeline.runAutoICP(cloudSource, cloudReference, 0.8);

    // 3. Đánh giá kết quả (Bước 4 trong Workflow)
    /*
    if (success) {
        EvaluationResult report = pipeline.evaluate(cloudSource, cloudReference);
        
        std::cout << "--- EVALUATION REPORT ---" << std::endl;
        std::cout << "Trimmed RMSE: " << report.trimmedRMSE << " mm" << std::endl;
        std::cout << "Inlier Count (N): " << report.inlierCount << std::endl;
        std::cout << "Verdict: " << report.verdict << std::endl;
    } else {
        std::cout << "ICP Failed to converge." << std::endl;
    }
    */

    return 0;
}