#include <ArduinoJson.h>
#include "json_joints.hpp"


JsonDocument getJsonJoints() {
    
    JsonDocument json_joints;

    JsonObject L0S1 = json_joints["L0S1"].to<JsonObject>();
    L0S1["leg"] = 0;
    L0S1["axis"] = 0;
    JsonObject L0S2 = json_joints["L0S2"].to<JsonObject>();
    L0S2["leg"] = 0;
    L0S2["axis"] = 1;
    JsonObject L0S3 = json_joints["L0S3"].to<JsonObject>();
    L0S3["leg"] = 0;
    L0S3["axis"] = 2;

    JsonObject L1S1 = json_joints["L1S1"].to<JsonObject>();
    L1S1["leg"] = 1;
    L1S1["axis"] = 0;
    JsonObject L1S2 = json_joints["L1S2"].to<JsonObject>();
    L1S2["leg"] = 1;
    L1S2["axis"] = 1;
    JsonObject L1S3 = json_joints["L1S3"].to<JsonObject>();
    L1S3["leg"] = 1;
    L1S3["axis"] = 2;

    JsonObject L2S1 = json_joints["L2S1"].to<JsonObject>();
    L2S1["leg"] = 2;
    L2S1["axis"] = 0;
    JsonObject L2S2 = json_joints["L2S2"].to<JsonObject>();
    L2S2["leg"] = 2;
    L2S2["axis"] = 1;
    JsonObject L2S3 = json_joints["L2S3"].to<JsonObject>();
    L2S3["leg"] = 2;
    L2S3["axis"] = 2;
    
    JsonObject L3S1 = json_joints["L3S1"].to<JsonObject>();
    L3S1["leg"] = 3;
    L3S1["axis"] = 0;
    JsonObject L3S2 = json_joints["L3S2"].to<JsonObject>();
    L3S2["leg"] = 3;
    L3S2["axis"] = 1;
    JsonObject L3S3 = json_joints["L3S3"].to<JsonObject>();
    L3S3["leg"] = 3;
    L3S3["axis"] = 2;

    JsonObject L4S1 = json_joints["L4S1"].to<JsonObject>();
    L4S1["leg"] = 4;
    L4S1["axis"] = 0;
    JsonObject L4S2 = json_joints["L4S2"].to<JsonObject>();
    L4S2["leg"] = 4;
    L4S2["axis"] = 1;
    JsonObject L4S3 = json_joints["L4S3"].to<JsonObject>();
    L4S3["leg"] = 4;
    L4S3["axis"] = 2;

    JsonObject L5S1 = json_joints["L5S1"].to<JsonObject>();
    L5S1["leg"] = 5;
    L5S1["axis"] = 0;
    JsonObject L5S2 = json_joints["L5S2"].to<JsonObject>();
    L5S2["leg"] = 5;
    L5S2["axis"] = 1;
    JsonObject L5S3 = json_joints["L5S3"].to<JsonObject>();
    L5S3["leg"] = 5;
    L5S3["axis"] = 2;

    return json_joints;

}
