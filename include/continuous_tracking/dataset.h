#ifndef OBJECT_TRACKING_ROS_DATASET_H
#define OBJECT_TRACKING_ROS_DATASET_H

#include <string>
#include <vector>

inline const std::vector<std::string>& getClassNames()
{
    const static std::vector<std::string> class_names = {"person",
                                                         "bicycle",
                                                         "car",
                                                         "motorcycle",
                                                         "airplane",
                                                         "bus",
                                                         "train",
                                                         "truck",
                                                         "boat",
                                                         "traffic light",
                                                         "fire hydrant",
                                                         "stop sign",
                                                         "parking meter",
                                                         "bench",
                                                         "bird",
                                                         "cat",
                                                         "dog",
                                                         "horse",
                                                         "sheep",
                                                         "cow",
                                                         "elephant",
                                                         "bear",
                                                         "zebra",
                                                         "giraffe",
                                                         "backpack",
                                                         "umbrella",
                                                         "handbag",
                                                         "tie",
                                                         "suitcase",
                                                         "frisbee",
                                                         "skis",
                                                         "snowboard",
                                                         "sports ball",
                                                         "kite",
                                                         "baseball bat",
                                                         "baseball glove",
                                                         "skateboard",
                                                         "surfboard",
                                                         "tennis racket",
                                                         "bottle",
                                                         "wine glass",
                                                         "cup",
                                                         "fork",
                                                         "knife",
                                                         "spoon",
                                                         "bowl",
                                                         "banana",
                                                         "apple",
                                                         "sandwich",
                                                         "orange",
                                                         "broccoli",
                                                         "carrot",
                                                         "hot dog",
                                                         "pizza",
                                                         "donut",
                                                         "cake",
                                                         "chair",
                                                         "couch",
                                                         "potted plant",
                                                         "bed",
                                                         "dining table",
                                                         "toilet",
                                                         "tv",
                                                         "laptop",
                                                         "mouse",
                                                         "remote",
                                                         "keyboard",
                                                         "cell phone",
                                                         "microwave",
                                                         "oven",
                                                         "toaster",
                                                         "sink",
                                                         "refrigerator",
                                                         "book",
                                                         "clock",
                                                         "vase",
                                                         "scissors",
                                                         "teddy bear",
                                                         "hair drier",
                                                         "toothbrush",
                                                         "banner",
                                                         "blanket",
                                                         "bridge",
                                                         "cardboard",
                                                         "counter",
                                                         "curtain",
                                                         "door-stuff",
                                                         "floor-wood",
                                                         "flower",
                                                         "fruit",
                                                         "gravel",
                                                         "house",
                                                         "light",
                                                         "mirror-stuff",
                                                         "net",
                                                         "pillow",
                                                         "platform",
                                                         "playingfield",
                                                         "railroad",
                                                         "river",
                                                         "road",
                                                         "roof",
                                                         "sand",
                                                         "sea",
                                                         "shelf",
                                                         "snow",
                                                         "stairs",
                                                         "tent",
                                                         "towel",
                                                         "wall-brick",
                                                         "wall-stone",
                                                         "wall-tile",
                                                         "wall-wood",
                                                         "water-other",
                                                         "window-blind",
                                                         "window-other",
                                                         "tree-merged",
                                                         "fence-merged",
                                                         "ceiling-merged",
                                                         "sky-other-merged",
                                                         "cabinet-merged",
                                                         "table-merged",
                                                         "floor-other-merged",
                                                         "pavement-merged",
                                                         "mountain-merged",
                                                         "grass-merged",
                                                         "dirt-merged",
                                                         "paper-merged",
                                                         "food-other-merged",
                                                         "building-other-merged",
                                                         "rock-merged",
                                                         "wall-other-merged",
                                                         "rug-merged",
                                                         "unknown"};
    return class_names;
}

int getClassCount()
{
    return static_cast<int>(getClassNames().size());
}

inline std::string getClassName(uint16_t label)
{
    return getClassNames()[label];
}

inline const std::vector<std::string>& getForegroundClassNames()
{
    const static std::vector<std::string> foreground_class_names = {
        "person",   "bicycle",     "car",        "motorcycle", "bus",          "train",   "truck",
        "bird",     "cat",         "dog",        "horse",      "sheep",        "cow",     "elephant",
        "bear",     "zebra",       "giraffe",    "backpack",   "umbrella",     "handbag", "tie",
        "suitcase", "sports ball", "skateboard", "surfboard",  "tennis racket"};
    return foreground_class_names;
}

inline const std::vector<uint16_t>& getForegroundClassLabels()
{
    static std::vector<uint16_t> foreground_class_labels;
    if (foreground_class_labels.empty())
    {
        const auto& class_names = getClassNames();
        const auto& foreground_class_names = getForegroundClassNames();
        for (const auto& foreground_class_name : foreground_class_names)
            foreground_class_labels.push_back(std::find(class_names.begin(), class_names.end(), foreground_class_name) -
                                              class_names.begin());
    }
    return foreground_class_labels;
}

inline bool isForegroundClass(uint16_t label)
{
    const auto& foreground_class_labels = getForegroundClassLabels();
    auto it = std::lower_bound(foreground_class_labels.begin(), foreground_class_labels.end(), label);
    return it != foreground_class_labels.end() && *it == label;
}

#endif // OBJECT_TRACKING_ROS_DATASET_H
