#include "inferenceworker.h"
#include "utils.h"

InferenceWorker::InferenceWorker(int frameWidth, int frameHeight, cv::Mat& img) {
    m_frameWidth = frameWidth;
    m_frameHeight = frameHeight;
    m_inputFrame = img;
    
    // Initialize ONNX Runtime
    initializeONNXRuntime();
    readClassNames();
}

InferenceWorker::~InferenceWorker() {
    clearInput();
}

void InferenceWorker::initializeONNXRuntime() {
    try {
        // Create ONNX Runtime environment
        m_env = std::make_unique<Ort::Env>(ORT_LOGGING_LEVEL_WARNING, "YOLOv11");
        
        // Create session options
        Ort::SessionOptions sessionOptions;
        sessionOptions.SetIntraOpNumThreads(std::thread::hardware_concurrency());
        sessionOptions.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
        
        // Enable CUDA if available (optional - remove if CPU only)
        // OrtCUDAProviderOptions cudaOptions;
        // sessionOptions.AppendExecutionProvider_CUDA(cudaOptions);
        
        // Create session
        m_session = std::make_unique<Ort::Session>(*m_env, L"deps/models/yolo12n_DynamicAxis.onnx", sessionOptions);
        
        // Get input/output info
        auto inputNamePtr = m_session->GetInputNameAllocated(0, m_allocator);
        auto outputNamePtr = m_session->GetOutputNameAllocated(0, m_allocator);
        m_inputName = std::string(inputNamePtr.get());
        m_outputName = std::string(outputNamePtr.get());
        
        // Get input shape
        auto inputTypeInfo = m_session->GetInputTypeInfo(0);
        auto inputTensorInfo = inputTypeInfo.GetTensorTypeAndShapeInfo();
        m_inputShape = inputTensorInfo.GetShape();
        
        // Set input dimensions (assuming NCHW format: [batch, channels, height, width])
		// batch, height and width are -1 for models that support dynamic batching
        m_inputHeight = 640; //m_inputShape[2];
        m_inputWidth = 640; //m_inputShape[3];
        
        LOG_INFO("ONNX Runtime initialized successfully with CPU and " << std::thread::hardware_concurrency() << "threads");
        LOG_INFO("Input shape: " << m_inputShape[0] << "x" << m_inputShape[1] << "x" << m_inputShape[2] << "x" << m_inputShape[3]);
        
    } catch (const Ort::Exception& e) {
        LOG_CRITICAL("ONNX Runtime initialization failed: " << e.what());
        throw std::runtime_error("Failed to initialize ONNX Runtime");
    }
}

std::vector<cv::Mat> InferenceWorker::splitImageIntoQuadrants(const cv::Mat& image) {
    std::vector<cv::Mat> quadrants;

    if (TILE_FACTOR == 2){
    int halfWidth = image.cols / 2;
    int halfHeight = image.rows / 2;

    // Top-left quadrant
    cv::Rect topLeft(0, 0, halfWidth, halfHeight);
    quadrants.push_back(image(topLeft));

    // Top-right quadrant
    cv::Rect topRight(halfWidth, 0, halfWidth, halfHeight);
    quadrants.push_back(image(topRight));

    // Bottom-left quadrant
    cv::Rect bottomLeft(0, halfHeight, halfWidth, halfHeight);
    quadrants.push_back(image(bottomLeft));

    // Bottom-right quadrant
    cv::Rect bottomRight(halfWidth, halfHeight, halfWidth, halfHeight);
    quadrants.push_back(image(bottomRight));

	return quadrants;
}

    
	// ensure that the image is divisible by TILE_FACTOR
	int tileWidth = image.cols / TILE_FACTOR;
	int tileHeight = image.rows / TILE_FACTOR;

    for (int i = 0; i < TILE_FACTOR; ++i) {
        for (int j = 0; j < TILE_FACTOR; ++j) {
            int x = j * tileWidth;
            int y = i * tileHeight;
            cv::Rect tileRect(x, y, tileWidth, tileHeight);
            if (x + tileWidth <= image.cols && y + tileHeight <= image.rows) {
                quadrants.push_back(image(tileRect));
				cv::String tileName = "tile_" + std::to_string(i) + "_" + std::to_string(j) + ".jpg";
                cv::imwrite(tileName, image(tileRect));
            }
        }
	}
    
    return quadrants;
}

std::vector<float> InferenceWorker::preprocessImage(const cv::Mat& image) {
    cv::Mat resized;
    cv::resize(image, resized, cv::Size(m_inputWidth, m_inputHeight));
    
    // Normalize and convert to float
    cv::Mat normalized;
    resized.convertTo(normalized, CV_32F, 1.0 / 255.0);
    
    // Convert HWC to CHW format
    std::vector<float> input(m_inputWidth * m_inputHeight * 3);
    std::vector<cv::Mat> channels(3);
    cv::split(normalized, channels);
    
    for (int c = 0; c < 3; ++c) {
        std::memcpy(input.data() + c * m_inputWidth * m_inputHeight, 
                   channels[c].data, m_inputWidth * m_inputHeight * sizeof(float));
    }
    
    return input;
}

std::vector<float> InferenceWorker::preprocessBatchedImages(const std::vector<cv::Mat>& images) {
    int batchSize = images.size();
    int inputSize = m_inputWidth * m_inputHeight * 3;
    std::vector<float> batchedInput(batchSize * inputSize);
    
    for (int i = 0; i < batchSize; ++i) {
        cv::Mat resized;
        cv::resize(images[i], resized, cv::Size(m_inputWidth, m_inputHeight));

        // Normalize and convert to float
        cv::Mat normalized;
        resized.convertTo(normalized, CV_32F, 1.0 / 255.0);

        // Convert HWC to CHW format
        std::vector<float> input(m_inputWidth * m_inputHeight * 3);
        std::vector<cv::Mat> channels(3);
        cv::split(normalized, channels);

        for (int c = 0; c < 3; ++c) {
            std::memcpy(batchedInput.data() + c * m_inputWidth * m_inputHeight + i * inputSize,
                       channels[c].data, m_inputWidth * m_inputHeight * sizeof(float));
		}
    }
    
    return batchedInput;
}

cv::Rect InferenceWorker::createBox(float cx, float cy, float w, float h, float scale_x, float scale_y) {
    // Convert from center coordinates to top-left coordinates
    float x1 = (cx - w / 2) * scale_x;
    float y1 = (cy - h / 2) * scale_y;
    float x2 = (cx + w / 2) * scale_x;
    float y2 = (cy + h / 2) * scale_y;
    
    // Clamp to image bounds
    x1 = std::max(0.0f, std::min(x1, static_cast<float>(m_inputFrame.cols)));
    y1 = std::max(0.0f, std::min(y1, static_cast<float>(m_inputFrame.rows)));
    x2 = std::max(0.0f, std::min(x2, static_cast<float>(m_inputFrame.cols)));
    y2 = std::max(0.0f, std::min(y2, static_cast<float>(m_inputFrame.rows)));
    
    return cv::Rect(cv::Point(x1, y1), cv::Point(x2, y2));
}

cv::Rect InferenceWorker::createBoxForQuadrant(float cx, float cy, float w, float h, float scale_x, float scale_y, int quadrant, int quadWidth, int quadHeight) {
    // Convert from center coordinates to top-left coordinates
    float x1 = (cx - w / 2) * scale_x;
    float y1 = (cy - h / 2) * scale_y;
    float x2 = (cx + w / 2) * scale_x;
    float y2 = (cy + h / 2) * scale_y;
    
    // Clamp to quadrant bounds
    x1 = std::max(0.0f, std::min(x1, static_cast<float>(quadWidth)));
    y1 = std::max(0.0f, std::min(y1, static_cast<float>(quadHeight)));
    x2 = std::max(0.0f, std::min(x2, static_cast<float>(quadWidth)));
    y2 = std::max(0.0f, std::min(y2, static_cast<float>(quadHeight)));

    if (TILE_FACTOR == 2) {
        // Adjust coordinates to original image based on quadrant
        int halfWidth = m_inputFrame.cols / 2;
        int halfHeight = m_inputFrame.rows / 2;
        switch (quadrant) {
            case 1: // Top-left
                // No adjustment needed
                break;
            case 2: // Top-right
                x1 += halfWidth;
                x2 += halfWidth;
                break;
            case 3: // Bottom-left
                y1 += halfHeight;
                y2 += halfHeight;
                break;
            case 4: // Bottom-right
                x1 += halfWidth;
                x2 += halfWidth;
                y1 += halfHeight;
                y2 += halfHeight;
                break;
        }
        return cv::Rect(cv::Point(x1, y1), cv::Point(x2, y2));
	}
    
    if (quadrant > 0 && quadrant <= 4) {
		x1 += (quadrant - 1) * quadWidth;
		x2 += (quadrant - 1) * quadWidth;
    }
    else if (quadrant > 4 && quadrant <= 8) {
		x1 += (quadrant - 4 - 1) * quadWidth;
		x2 += (quadrant - 4 - 1) * quadWidth;
		y1 += quadHeight;
		y2 += quadHeight;
    }
    else if (quadrant > 8 && quadrant <= 12) {
		x1 += (quadrant - 8 - 1) * quadWidth;
		x2 += (quadrant - 8 - 1) * quadWidth;
		y1 += 2 * quadHeight;
		y2 += 2 * quadHeight;
    }
    else if (quadrant > 12 && quadrant <= 16) {
		x1 += (quadrant - 12 - 1) * quadWidth;
		x2 += (quadrant - 12 - 1) * quadWidth;
		y1 += 3 * quadHeight;
		y2 += 3 * quadHeight;
    }

    return cv::Rect(cv::Point(x1, y1), cv::Point(x2, y2));
}

std::vector<cv::Rect> InferenceWorker::drawBoxes(std::vector<cv::Rect>& boxes, std::vector<int>& classIds, 
                                   std::vector<float>& confidences, std::vector<int>& indices) {
	std::vector<cv::Rect> centroids;
	centroids.reserve(indices.size());

    for (int i = 0; i < indices.size(); i++) {
        int idx = indices[i];
        cv::Rect box = boxes[idx];
        int class_id = classIds[idx];
        
        // Draw the bounding box
        cv::rectangle(m_inputFrame, box, cv::Scalar(0, 0, 0), 2);
        
        // Draw the class label
        std::string label = m_classNames[class_id] + " " + std::to_string(confidences[idx]).substr(0, 4);
        
        // Get text size for background
        int baseline = 0;
        cv::Size textSize = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseline);
        
        // Draw background rectangle
        /*cv::rectangle(m_inputFrame,
                     cv::Point(box.x, box.y - textSize.height - baseline),
                     cv::Point(box.x + textSize.width, box.y),
                     cv::Scalar(0, 255, 0), -1);*/
        
        // Draw text
        cv::putText(m_inputFrame, label, cv::Point(box.x, box.y - baseline), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 2);

		centroids.push_back(box);
    }

	return centroids;
}

void InferenceWorker::readClassNames() {
    std::string classNamesPath = "deps/models/class_names.txt";
    if (!std::filesystem::exists(classNamesPath)) {
        m_classNames.push_back("ce");
        m_classNames.push_back("clump");
        return;
    }

    std::ifstream classFile(classNamesPath);
    if (!classFile.is_open()) return;

    std::string line;
    while (std::getline(classFile, line)) {
        if (!line.empty()) {
            m_classNames.push_back(line);
        }
    }
    classFile.close();
}

void InferenceWorker::runModel(cv::Mat& input) {
    try {
        // Preprocess image
		START_TIMER(preprocess);
        std::vector<float> inputData = preprocessImage(input);
		END_TIMER(preprocess);
        
        // Create input tensor
        std::vector<int64_t> inputShape = {1, 3, m_inputHeight, m_inputWidth};
        Ort::Value inputTensor = Ort::Value::CreateTensor<float>(
            m_memoryInfo, inputData.data(), inputData.size(), inputShape.data(), inputShape.size());
        
        // Run inference
        const char* inputNames[] = { m_inputName.c_str() };
        const char* outputNames[] = { m_outputName.c_str() };
        
		START_TIMER(inference);
        std::vector<Ort::Value> outputTensors = m_session->Run(
            Ort::RunOptions{nullptr}, inputNames, &inputTensor, 1, outputNames, 1);
		END_TIMER(inference);
        
        // Process output
		START_TIMER(postprocess);
        processOutput(outputTensors[0], input);
		END_TIMER(postprocess);
        
    } catch (const Ort::Exception& e) {
        LOG_CRITICAL("ONNX Runtime inference failed: " << e.what());
    }
}

std::vector<cv::Rect> InferenceWorker::runBatchedModel(cv::Mat& input) {
    try {
        // Split image into quadrants
        START_TIMER(split);
        std::vector<cv::Mat> quadrants = splitImageIntoQuadrants(input);
        END_TIMER(split);
        
        // Preprocess all quadrants
        START_TIMER(preprocess);
        std::vector<float> inputData = preprocessBatchedImages(quadrants);
        END_TIMER(preprocess);
        
        // Create input tensor with batch size 4
        std::vector<int64_t> inputShape = {TILE_FACTOR * TILE_FACTOR, 3, m_inputHeight, m_inputWidth};
        Ort::Value inputTensor = Ort::Value::CreateTensor<float>(
            m_memoryInfo, inputData.data(), inputData.size(), inputShape.data(), inputShape.size());
        
        // Run inference
        const char* inputNames[] = { m_inputName.c_str() };
        const char* outputNames[] = { m_outputName.c_str() };
        
        START_TIMER(inference);
        std::vector<Ort::Value> outputTensors = m_session->Run(
            Ort::RunOptions{nullptr}, inputNames, &inputTensor, 1, outputNames, 1);
        END_TIMER(inference);
        
        // Process output
        START_TIMER(postprocess);
        auto ret = processBatchedOutput(outputTensors[0], input);
        END_TIMER(postprocess);

		return ret;
        
    } catch (const Ort::Exception& e) {
        LOG_CRITICAL("ONNX Runtime inference failed: " << e.what());
    }
}

std::vector<cv::Rect> InferenceWorker::shortestPath(std::vector<cv::Rect>& centroids) {
    if (centroids.empty()) {
		LOG_INFO("No centroids found for shortest path calculation.");
        return {};
    }

    const size_t n = centroids.size();
    std::vector<cv::Rect> path;
    std::vector<bool> used(n, false);
    path.reserve(n);

    path.push_back(centroids[0]);
    used[0] = true;

    for (size_t count = 1; count < n; ++count) {
        const cv::Rect& current = path.back();
        float min_dist_sq = std::numeric_limits<float>::max();
        size_t nearest_idx = 0;

        for (size_t i = 0; i < n; ++i) {
            if (!used[i]) {
                // Direct calculation using OpenCV cv::Point's optimized storage
                float dx = current.x - centroids[i].x;
                float dy = current.y - centroids[i].y;
                float dist_sq = dx * dx + dy * dy;

                if (dist_sq < min_dist_sq) {
                    min_dist_sq = dist_sq;
                    nearest_idx = i;
                }
            }
        }

        path.push_back(centroids[nearest_idx]);
        used[nearest_idx] = true;
    }

    return path;
}


void InferenceWorker::processOutput(Ort::Value& output, cv::Mat& originalImage) {
    // Get output tensor info
    auto outputShape = output.GetTensorTypeAndShapeInfo().GetShape();
    float* outputData = output.GetTensorMutableData<float>();

	// float array to vector
    size_t output_size = 1;
    for (auto dim : outputShape) {
        output_size *= dim;
    }
    std::vector<float> outputVec(outputData, outputData + output_size);
    
    // YOLOv11 output format: [batch, predictions, (x, y, w, h, class_scores...)]
    int numPredictions = outputShape[2];
    int predictionSize = outputShape[1];
    
    std::vector<float> confidences;
    std::vector<cv::Rect> boxes;
    std::vector<int> classIds;
    
    // Calculate scale factors
    float scaleX = static_cast<float>(originalImage.cols) / m_inputWidth;
    float scaleY = static_cast<float>(originalImage.rows) / m_inputHeight;
    
    for (int i = 0; i < numPredictions; ++i) {
        // For output shape [1, 6, 8400], data is organized as:
        // output[0*8400 + i] = x_center
        // output[1*8400 + i] = y_center  
        // output[2*8400 + i] = width
        // output[3*8400 + i] = height
        // output[4*8400 + i] = confidence1
        // output[5*8400 + i] = confidence2

        float* prediction = outputData + i * numPredictions;
        
        // Extract coordinates
        float cx = outputVec[0 * numPredictions + i];
        float cy = outputVec[1 * numPredictions + i];
        float w = outputVec[2 * numPredictions + i];
        float h = outputVec[3 * numPredictions + i];
        
        // Find max class score
        float maxClassScore = 0.0f;
        int maxClassId = 0;
        
        for (int j = 4; j < predictionSize; ++j) {
            if (outputVec[j * numPredictions + i] > maxClassScore) {
                maxClassScore = outputVec[j * numPredictions + i];
                maxClassId = j - 4;
            }
        }
        
        // Filter by confidence threshold
        if (maxClassScore > CONFIDENCE_THRESHOLD) {
            confidences.push_back(maxClassScore);
            classIds.push_back(maxClassId);
            
            cv::Rect box = createBox(cx, cy, w, h, scaleX, scaleY);
            boxes.push_back(box);
        }
    }
    
    // Apply Non-Maximum Suppression
    std::vector<int> indices;
    cv::dnn::NMSBoxes(boxes, confidences, CONFIDENCE_THRESHOLD, OVERLAP_THRESHOLD, indices);
    
    // Draw results
    drawBoxes(boxes, classIds, confidences, indices);
	LOG_INFO("Valid detections found - " << indices.size());
}

std::vector<cv::Rect> InferenceWorker::processBatchedOutput(Ort::Value& output, cv::Mat& originalImage) {
    // Get output tensor info
    auto outputShape = output.GetTensorTypeAndShapeInfo().GetShape();
    float* outputData = output.GetTensorMutableData<float>();

    // YOLOv11 output format: [batch, predictions, (x, y, w, h, class_scores...)]
    int batchSize = outputShape[0];
    int predictionSize = outputShape[1];
    int numPredictions = outputShape[2];
    
    std::vector<float> allConfidences;
    std::vector<cv::Rect> allBoxes;
    std::vector<int> allClassIds;
    std::vector<cv::Point> allCentroids;

    float scaleX, scaleY;
	int tileWidth = 0, tileHeight = 0;
    
    // Calculate scale factors for each quadrant
    if (TILE_FACTOR == 2) {
        // For 2x2 tiling, calculate half dimensions
        tileWidth = originalImage.cols / 2;
        tileHeight = originalImage.rows / 2;
        scaleX = static_cast<float>(tileWidth) / m_inputWidth;
        scaleY = static_cast<float>(tileHeight) / m_inputHeight;
	}
    else if (TILE_FACTOR == 4) {
        // Calculate tile dimensions
        tileWidth = originalImage.cols / TILE_FACTOR;
        tileHeight = originalImage.rows / TILE_FACTOR;
        scaleX = static_cast<float>(tileWidth) / m_inputWidth;
        scaleY = static_cast<float>(tileHeight) / m_inputHeight;
    }
    
    for (int b = 0; b < batchSize; ++b) {
        // Calculate offset for this batch
        int batchOffset = b * predictionSize * numPredictions;
        
        for (int i = 0; i < numPredictions; ++i) {
            // Extract coordinates for this prediction in this batch
            float cx = outputData[batchOffset + 0 * numPredictions + i];
            float cy = outputData[batchOffset + 1 * numPredictions + i];
            float w = outputData[batchOffset + 2 * numPredictions + i];
            float h = outputData[batchOffset + 3 * numPredictions + i];
            
            // Find max class score
            float maxClassScore = 0.0f;
            int maxClassId = 0;
            
            for (int j = 4; j < predictionSize; ++j) {
                float score = outputData[batchOffset + j * numPredictions + i];
                if (score > maxClassScore) {
                    maxClassScore = score;
                    maxClassId = j - 4;
                }
            }
            
            // Filter by confidence threshold
            if (maxClassScore > CONFIDENCE_THRESHOLD) {
                allConfidences.push_back(maxClassScore);
                allClassIds.push_back(maxClassId);
                
                //cv::Rect box = createBoxForQuadrant(cx, cy, w, h, scaleX, scaleY, b, halfWidth, halfHeight);
                cv::Rect box = createBoxForQuadrant(cx, cy, w, h, scaleX, scaleY, b + 1, tileWidth, tileHeight);
                allBoxes.push_back(box);
				allCentroids.push_back(cv::Point(cx * scaleX, cy * scaleY));
            }
        }
    }
    
    // Apply global NMS to remove overlapping detections between quadrants
    std::vector<int> finalIndices;
    cv::dnn::NMSBoxes(allBoxes, allConfidences, CONFIDENCE_THRESHOLD, OVERLAP_THRESHOLD, finalIndices);

    if (finalIndices.empty()) {
        LOG_INFO("No valid detections found after NMS.");
        return {};
	}

    LOG_INFO("Valid detections found - " << finalIndices.size());
    
    // Draw results
    std::vector<cv::Rect> centroids = drawBoxes(allBoxes, allClassIds, allConfidences, finalIndices);
	std::vector<cv::Rect> path = shortestPath(centroids);
    
    for (int i = 0; i < path.size() - 1; ++i) {
        cv::line(m_inputFrame,  (path[i].tl() + path[i].br()) * 0.5, (path[i + 1].tl() + path[i + 1].br()) * 0.5, cv::Scalar(255, 0, 0), 2);
	}

    return path;
}

void InferenceWorker::predict() {
    QMutexLocker locker(&m_mutex);
    
    if (m_inputFrame.empty()) {
        LOG_WARNING("Input frame is empty. Cannot run inference.");
        return;
	}

	START_TIMER(predictionTotal);
    // Use batched inference for better performance
    //runModel(m_inputFrame);
    std::vector<cv::Rect> boxCentroids = runBatchedModel(m_inputFrame);
	END_TIMER(predictionTotal);
    
    emit frameProcessed(m_inputFrame, boxCentroids);
}