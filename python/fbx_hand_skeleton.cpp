#include <fbxsdk.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <string>
#include <vector>

struct Vec3 {
    double x;
    double y;
    double z;
};

struct Segment {
    int start;
    int end;
};

struct Track {
    std::string name;
    FbxNode* node;
    std::vector<FbxQuaternion> rotations;
};

double dot(const Vec3& a, const Vec3& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

Vec3 subtract(const Vec3& a, const Vec3& b) {
    return {a.x - b.x, a.y - b.y, a.z - b.z};
}

Vec3 add(const Vec3& a, const Vec3& b) {
    return {a.x + b.x, a.y + b.y, a.z + b.z};
}

Vec3 multiply(const Vec3& value, double scalar) {
    return {value.x * scalar, value.y * scalar, value.z * scalar};
}

Vec3 normalize(const Vec3& value) {
    double length = std::sqrt(dot(value, value));
    return length > 1e-12 ? multiply(value, 1.0 / length) : Vec3{0.0, 0.0, 0.0};
}

Vec3 cross(const Vec3& a, const Vec3& b) {
    return {
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    };
}

double quaternionDot(const FbxQuaternion& a, const FbxQuaternion& b) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
}

FbxQuaternion normalizedQuaternion(FbxQuaternion value) {
    double length = std::sqrt(quaternionDot(value, value));
    for (int i = 0; i < 4; ++i) value[i] /= length;
    return value;
}

std::vector<double> gaussianKernel(double sigma) {
    int radius = static_cast<int>(std::ceil(3.0 * sigma));
    std::vector<double> kernel(2 * radius + 1);
    double sum = 0.0;
    for (int i = -radius; i <= radius; ++i) {
        kernel[i + radius] = std::exp(-0.5 * i * i / (sigma * sigma));
        sum += kernel[i + radius];
    }
    for (double& value : kernel) value /= sum;
    return kernel;
}

void smoothQuaternions(std::vector<FbxQuaternion>& values, double sigma) {
    if (values.empty()) return;
    for (size_t i = 1; i < values.size(); ++i) {
        if (quaternionDot(values[i - 1], values[i]) < 0.0) {
            for (int component = 0; component < 4; ++component) values[i][component] *= -1.0;
        }
    }

    std::vector<double> kernel = gaussianKernel(sigma);
    int radius = static_cast<int>(kernel.size() / 2);
    std::vector<FbxQuaternion> smoothed = values;
    for (int i = 0; i < static_cast<int>(values.size()); ++i) {
        FbxQuaternion sum(0.0, 0.0, 0.0, 0.0);
        double weightSum = 0.0;
        for (int offset = -radius; offset <= radius; ++offset) {
            int index = i + offset;
            if (index < 0 || index >= static_cast<int>(values.size())) continue;
            double weight = kernel[offset + radius];
            for (int component = 0; component < 4; ++component) sum[component] += weight * values[index][component];
            weightSum += weight;
        }
        for (int component = 0; component < 4; ++component) sum[component] /= weightSum;
        smoothed[i] = normalizedQuaternion(sum);
    }
    values = std::move(smoothed);
}

void smoothValues(std::vector<double>& values, double sigma) {
    std::vector<double> kernel = gaussianKernel(sigma);
    int radius = static_cast<int>(kernel.size() / 2);
    std::vector<double> smoothed = values;
    for (int i = 0; i < static_cast<int>(values.size()); ++i) {
        double sum = 0.0;
        double weightSum = 0.0;
        for (int offset = -radius; offset <= radius; ++offset) {
            int index = i + offset;
            if (index < 0 || index >= static_cast<int>(values.size())) continue;
            double weight = kernel[offset + radius];
            sum += weight * values[index];
            weightSum += weight;
        }
        smoothed[i] = sum / weightSum;
    }
    values = std::move(smoothed);
}

double angularSpeed(const FbxQuaternion& a, const FbxQuaternion& b, double deltaTime) {
    FbxQuaternion normalizedA = normalizedQuaternion(a);
    FbxQuaternion normalizedB = normalizedQuaternion(b);
    double quaternionProduct = std::clamp(std::abs(quaternionDot(normalizedA, normalizedB)), 0.0, 1.0);
    return 2.0 * std::acos(quaternionProduct) / deltaTime;
}

std::vector<Segment> findSegments(const std::vector<Track>& tracks, double fps) {
    int frameCount = static_cast<int>(tracks.front().rotations.size());
    double deltaTime = 1.0 / fps;
    std::vector<double> acceleration(frameCount, 0.0);
    for (int frame = 1; frame < frameCount - 1; ++frame) {
        double strongest = 0.0;
        for (const Track& track : tracks) {
            double before = angularSpeed(track.rotations[frame - 1], track.rotations[frame], deltaTime);
            double after = angularSpeed(track.rotations[frame], track.rotations[frame + 1], deltaTime);
            double current = (after - before) / deltaTime;
            if (std::abs(current) > std::abs(strongest)) strongest = current;
        }
        acceleration[frame] = strongest;
    }
    acceleration.front() = acceleration[1];
    acceleration.back() = acceleration[frameCount - 2];
    smoothValues(acceleration, 1.5);

    double mean = std::accumulate(acceleration.begin(), acceleration.end(), 0.0) / frameCount;
    double variance = 0.0;
    for (double value : acceleration) variance += (value - mean) * (value - mean);
    double threshold = std::max(0.5 * std::sqrt(variance / frameCount), 0.25);

    struct Extremum { int index; bool peak; };
    std::vector<Extremum> found;
    for (int i = 1; i < frameCount - 1; ++i) {
        if (acceleration[i] > threshold && acceleration[i] > acceleration[i - 1] && acceleration[i] >= acceleration[i + 1]) {
            found.push_back({i, true});
        } else if (acceleration[i] < -threshold && acceleration[i] < acceleration[i - 1] && acceleration[i] <= acceleration[i + 1]) {
            found.push_back({i, false});
        }
    }

    std::vector<Extremum> extrema;
    for (const Extremum& current : found) {
        if (!extrema.empty() && extrema.back().peak == current.peak) {
            bool stronger = current.peak
                ? acceleration[current.index] > acceleration[extrema.back().index]
                : acceleration[current.index] < acceleration[extrema.back().index];
            if (stronger) extrema.back() = current;
        } else {
            extrema.push_back(current);
        }
    }
    if (extrema.empty()) return {{0, frameCount - 1}};

    std::vector<Segment> segments;
    auto addSegment = [&](int start, int end) {
        if (end - start + 1 >= 2) segments.push_back({start, end});
    };
    size_t index = 0;
    if (extrema.front().peak) {
        addSegment(0, extrema.front().index);
        index = 1;
    }
    for (; index < extrema.size(); index += 2) {
        addSegment(extrema[index].index, index + 1 < extrema.size() ? extrema[index + 1].index : frameCount - 1);
    }
    return segments;
}

Vec3 positionAt(FbxNode* node, const FbxTime& time) {
    FbxVector4 position = node->EvaluateGlobalTransform(time).GetT();
    return {position[0], position[1], position[2]};
}

Vec3 toHandFrame(const Vec3& position, const Vec3& origin, const Vec3& xAxis, const Vec3& yAxis, const Vec3& zAxis) {
    Vec3 offset = subtract(position, origin);
    return {dot(offset, xAxis), dot(offset, yAxis), dot(offset, zAxis)};
}

void printVector(const Vec3& value) {
    std::cout << '[' << value.x << ',' << value.y << ',' << value.z << ']';
}

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "Usage: fbx_hand_skeleton.exe <animation.fbx>\n";
        return 1;
    }

    FbxManager* manager = FbxManager::Create();
    manager->SetIOSettings(FbxIOSettings::Create(manager, IOSROOT));
    FbxImporter* importer = FbxImporter::Create(manager, "");
    if (!importer->Initialize(argv[1], -1, manager->GetIOSettings())) {
        std::cerr << importer->GetStatus().GetErrorString() << '\n';
        return 2;
    }
    FbxScene* scene = FbxScene::Create(manager, "scene");
    if (!importer->Import(scene)) return 3;
    importer->Destroy();

    FbxAnimStack* stack = scene->GetSrcObject<FbxAnimStack>(0);
    if (!stack) return 4;
    scene->SetCurrentAnimationStack(stack);
    FbxTime::EMode timeMode = scene->GetGlobalSettings().GetTimeMode();
    double fps = FbxTime::GetFrameRate(timeMode);
    FbxTimeSpan timeSpan = stack->GetLocalTimeSpan();
    FbxLongLong firstFrame = timeSpan.GetStart().GetFrameCount(timeMode);
    FbxLongLong lastFrame = timeSpan.GetStop().GetFrameCount(timeMode);
    int frameCount = static_cast<int>(lastFrame - firstFrame + 1);

    std::vector<std::string> segmentationBoneNames = {
        "RightHandThumb1", "RightHandThumb2", "RightHandThumb3",
        "RightHandIndex", "RightHandIndex1", "RightHandIndex2", "RightHandIndex3",
        "RightHand", "RightHandMiddle1", "RightHandMiddle2", "RightHandMiddle3",
        "RightHandRing", "RightHandRing1", "RightHandRing2", "RightHandRing3",
        "RightHandPinky", "RightHandPinky1", "RightHandPinky2", "RightHandPinky3"
    };
    std::vector<Track> tracks;
    for (const std::string& name : segmentationBoneNames) {
        FbxNode* node = scene->FindNodeByName(name.c_str());
        if (!node) continue;
        tracks.push_back({name, node, {}});
        tracks.back().rotations.reserve(frameCount);
        for (FbxLongLong frame = firstFrame; frame <= lastFrame; ++frame) {
            FbxTime time;
            time.SetFrame(frame, timeMode);
            tracks.back().rotations.push_back(node->EvaluateLocalTransform(time).GetQ());
        }
        smoothQuaternions(tracks.back().rotations, 2.0);
    }
    if (tracks.empty()) return 5;

    std::vector<Segment> segments = findSegments(tracks, fps);
    std::vector<std::pair<std::string, std::string>> jointNames = {
        {"thumb_metacarpal", "RightHandThumb1"}, {"thumb_proximal", "RightHandThumb2"},
        {"thumb_distal", "RightHandThumb3"}, {"thumb_tip", "RightHandThumb4"},
        {"index_metacarpal", "RightHandIndex"}, {"index_proximal", "RightHandIndex1"},
        {"index_middle", "RightHandIndex2"}, {"index_distal", "RightHandIndex3"}, {"index_tip", "RightHandIndex4"},
        {"middle_metacarpal", "RightHand"}, {"middle_proximal", "RightHandMiddle1"},
        {"middle_middle", "RightHandMiddle2"}, {"middle_distal", "RightHandMiddle3"}, {"middle_tip", "RightHandMiddle4"},
        {"ring_metacarpal", "RightHandRing"}, {"ring_proximal", "RightHandRing1"},
        {"ring_middle", "RightHandRing2"}, {"ring_distal", "RightHandRing3"}, {"ring_tip", "RightHandRing4"},
        {"pinky_metacarpal", "RightHandPinky"}, {"pinky_proximal", "RightHandPinky1"},
        {"pinky_middle", "RightHandPinky2"}, {"pinky_distal", "RightHandPinky3"}, {"pinky_tip", "RightHandPinky4"}
    };

    std::cout << std::setprecision(10) << '{';
    std::string baseName = std::filesystem::path(argv[1]).filename().string();
    for (size_t segmentIndex = 0; segmentIndex < segments.size(); ++segmentIndex) {
        if (segmentIndex > 0) std::cout << ',';
        Segment segment = segments[segmentIndex];
        int length = segment.end - segment.start + 1;
        int start = segment.start + length / 4;
        int end = segment.end - length / 4;
        std::cout << '"' << baseName << '_' << segmentIndex << "\":{";
        for (size_t jointIndex = 0; jointIndex < jointNames.size(); ++jointIndex) {
            if (jointIndex > 0) std::cout << ',';
            FbxNode* joint = scene->FindNodeByName(jointNames[jointIndex].second.c_str());
            Vec3 sum{0.0, 0.0, 0.0};
            int used = 0;
            for (int frameIndex = start; frameIndex <= end; ++frameIndex) {
                FbxTime time;
                time.SetFrame(firstFrame + frameIndex, timeMode);
                FbxNode* middleMetacarpal = scene->FindNodeByName("RightHand");
                Vec3 origin = positionAt(middleMetacarpal, time);
                Vec3 zAxis = normalize(subtract(positionAt(scene->FindNodeByName("RightHandMiddle1"), time), origin));
                Vec3 ySeed = subtract(positionAt(scene->FindNodeByName("RightHandIndex"), time), positionAt(scene->FindNodeByName("RightHandPinky"), time));
                Vec3 yAxis = normalize(subtract(ySeed, multiply(zAxis, dot(ySeed, zAxis))));
                Vec3 xAxis = normalize(cross(yAxis, zAxis));
                if (!joint || dot(xAxis, xAxis) < 0.5 || dot(yAxis, yAxis) < 0.5 || dot(zAxis, zAxis) < 0.5) continue;
                sum = add(sum, toHandFrame(positionAt(joint, time), origin, xAxis, yAxis, zAxis));
                ++used;
            }
            std::cout << '"' << jointNames[jointIndex].first << "\":";
            if (used == 0) std::cout << "null";
            else printVector(multiply(sum, 1.0 / used));
        }
        std::cout << '}';
    }
    std::cout << "}\n";
    manager->Destroy();
    return 0;
}
