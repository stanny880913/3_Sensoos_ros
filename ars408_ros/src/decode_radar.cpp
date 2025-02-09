#include <ros/ros.h>
#include <ros/console.h>
#include <can_msgs/Frame.h>
#include <std_msgs/String.h>
#include <std_msgs/Header.h>
#include <ars408_ros/ARS408_CAN.h>
#include <ars408_msg/RadarPoints.h>

// #define PRINT_SOCKET
// #define PRINT_RADAR_STATE
// #define PRINT_VERSION
// #define PRINT_FILTER_CONFIG
#define EFFECTIVE_RANGE

class RadarDecoder {
    public:
        RadarDecoder(std::string radar_name);
    
    private:
        ros::NodeHandle node_handle;
        std::map<std::string, ros::Subscriber> subs;
        std::map<std::string, ros::Publisher> pubs;

        std::map<int, ARS408::Object> objects_map;
        std::map<int, ARS408::Cluster> clusters_map;
        ARS408::ObjectStatus object_status;
        ARS408::ClusterStatus cluster_status;

        uint32_t seq;

	std::string radar_name;

        void cantopic_callback(const can_msgs::Frame::ConstPtr& msg);
};

RadarDecoder::RadarDecoder(std::string radar_name) {
    this->radar_name = radar_name;
    node_handle = ros::NodeHandle();
    subs["received_messages"] = node_handle.subscribe("/radar/" + radar_name + "/received_messages", 1000, &RadarDecoder::cantopic_callback, this);

    // std::vector<std::string> publish_topic_names{ "decoded_messages", "info_201", "info_700", "cluster_status", "object_status" };
    // for (const auto& i : publish_topic_names) {
    //     pubs[i] = node_handle.advertise<ars408_msg::RadarPoints>("/radar/" + radar_name + "/" + i, 1);
    // }

    pubs["decoded_messages"] = node_handle.advertise<ars408_msg::RadarPoints>("/radar/" + radar_name + "/" + "decoded_messages", 1);
    // pubs["object_status"] = node_handle.advertise<std_msgs::String>("/radar/" + radar_name + "/" + "object_status", 1);
    pubs["cluster_status"] = node_handle.advertise<std_msgs::String>("/radar/" + radar_name + "/" + "cluster_status", 1);
    pubs["info_201"] = node_handle.advertise<std_msgs::String>("/radar/" + radar_name + "/" + "info_201", 1);
    pubs["info_700"] = node_handle.advertise<std_msgs::String>("/radar/" + radar_name + "/" + "info_700", 1);
}

void RadarDecoder::cantopic_callback(const can_msgs::Frame::ConstPtr& msg) {
    #pragma region ShowData
    #ifdef PRINT_SOCKET
    if (msg->is_error)
    {
        std::cout << "E " << std::hex << msg->id << "#" << std::endl;
    }
    else if (msg->is_extended)
    {
        std::cout << "e " << std::hex << msg->id << "#" << std::endl;
    }
    else if (msg->is_rtr)
    {
        std::cout << "R " << std::hex << msg->id << "#" << std::endl;
    }
    else
    {
        std::stringstream getMessage;
        getMessage << "s " << std::hex << msg->id << "#";
        for (int i = 0; i < msg->dlc; ++i)
        {
            getMessage << " " << std::setfill('0') << std::setw(2) << std::hex << (unsigned int)msg->data[i];
        }
        getMessage << std::endl;
        std::cout << getMessage.str();
    }
    #endif
    #pragma endregion


    #pragma region radar
    if (msg->id == 0x201)
    {
        ARS408::RadarState radar;
        radar.NVMwriteStatus    = msg->data[0] >> 7;
        radar.NVMReadStatus     = (msg->data[0] & 0b01000000) >> 6;
        radar.MaxDistanceCfg    = ((msg->data[1] << 2) | (msg->data[2] >> 6)) * 2;
        radar.Persistent_Error  = (msg->data[2] & 0b00100000) >> 5;
        radar.Interference      = (msg->data[2] & 0b00010000) >> 4;
        radar.Temperature_Error = (msg->data[2] & 0b00001000) >> 3;
        radar.Temporary_Error   = (msg->data[2] & 0b00000100) >> 2;
        radar.Voltage_Error     = (msg->data[2] & 0b00000010) >> 1;
        radar.Persistent_Error  = (msg->data[2] & 0b00000010) >> 1;
        radar.RadarPowerCfg     = ((msg->data[3] & 0b00000011) << 1) | (msg->data[4] >> 7);
        radar.SortIndex         = (msg->data[4] & 0b01110000) >> 4;
        radar.SensorID          = msg->data[4] & 0b00000111;
        radar.MotionRxState     = msg->data[5] >> 6;
        radar.SendExtInfoCfg    = (msg->data[5] & 0b00100000) >> 5;
        radar.SendQualityCfg    = (msg->data[5] & 0b00010000) >> 4;
        radar.OutputTypeCfg     = (msg->data[5] & 0b00001100) >> 2;
        radar.CtrlRelayCfg      = (msg->data[5] & 0b00000010) >> 1;
        radar.InvalidClusters   = msg->data[6];
        radar.RCS_Threshold     = (msg->data[7] & 0b00011100) >> 2;

        std::stringstream info_201;
        info_201 << "---- Error info ----" << std::endl;
        info_201 << "NVM Read Status   : " << (radar.NVMReadStatus  == 0 ? "failed" : "successful") << std::endl;
        info_201 << "NVM Write Status  : " << (radar.NVMwriteStatus == 0 ? "failed" : "successful") << std::endl;
        info_201 << "Persistent Error  : " << (radar.Persistent_Error  == 0 ? "No error" : "Error active") << std::endl;
        info_201 << "Temperature Error : " << (radar.Temperature_Error == 0 ? "No error" : "Error active") << std::endl;
        info_201 << "Temporary Error   : " << (radar.Temporary_Error   == 0 ? "No error" : "Error active") << std::endl;
        info_201 << "Voltage Error     : " << (radar.Voltage_Error     == 0 ? "No error" : "Error active") << std::endl;

        info_201 << "------ Config ------" << std::endl;
        info_201 << "Sort Index        : " << (radar.SortIndex == 0 ? "no sorting" : radar.SortIndex == 1 ? "sorted by range" : "sorted by RCS") << std::endl;
        info_201 << "Send ExtInfo Cfg  : " << (radar.SendExtInfoCfg == 0 ? "inactive" : "active") << std::endl;
        info_201 << "Send Quality Cfg  : " << (radar.SendQualityCfg == 0 ? "inactive" : "active") << std::endl;
        info_201 << "Output Type Cfg   : " << (radar.OutputTypeCfg == 0 ? "none" : radar.OutputTypeCfg == 1 ? "send objects" : "send clusters") << std::endl;
        info_201 << "RadarPower Cfg    : " << ARS408::RadarPowerCfg[radar.RadarPowerCfg] << std::endl;
        info_201 << "Sensor ID         : " << radar.SensorID << std::endl;
        info_201 << "MaxDistanceCfg    : " << std::dec << radar.MaxDistanceCfg << std::endl;
        info_201 << "RCS Threshold     : " << (radar.RCS_Threshold == 0 ? "Standard" : "High sensitivity") << std::endl;
        info_201 << "Invalid Clusters  : " << std::setfill('0') << std::setw(2) << std::hex << radar.InvalidClusters << std::dec << " (Available only for SRR308)" << std::endl;
        info_201 << "Ctrl Relay Cfg    : " << (radar.CtrlRelayCfg == 0 ? "inactive" : "active") << std::endl;

        info_201 << "------ Others ------" << std::endl;
        info_201 << "MotionRxState     : " << ARS408::MotionRxState[radar.MotionRxState] << std::endl;
        info_201 << "Interference      : " << (radar.Interference == 0 ? "No interference" : "Interference detected") << std::endl;

        #ifdef PRINT_RADAR_STATE
        std::cout << info_201.str();
        #endif

        std_msgs::String str_201;
        str_201.data = info_201.str();
        pubs["info_201"].publish(str_201);
    }
    else if (msg->id == 0x700)
    {
        int Version_MajorRelease = msg->data[0];
        int Version_MinorRelease = msg->data[1];
        int Version_PatchLevel = msg->data[2];
        int Version_ExtendedRange = msg->data[2] & 0b00000010;
        int Version_CountryCode = msg->data[2] & 0b00000001;

        std::stringstream info_700;
        info_700 << "Version_MajorRelease  : " << Version_MajorRelease << std::endl;
        info_700 << "Version_MinorRelease  : " << Version_MinorRelease << std::endl;
        info_700 << "Version_PatchLevel    : " << Version_PatchLevel << std::endl;
        info_700 << "Version_ExtendedRange : " << Version_ExtendedRange << std::endl;
        info_700 << "Version_CountryCode   : " << Version_CountryCode << std::endl;

        #ifdef PRINT_VERSION
        std::cout << info_700.str();
        #endif

        std_msgs::String str_700;
        str_700.data = info_700.str();
        pubs["info_700"].publish(str_700);
    }
    #pragma endregion

    #pragma region Cluster
    if (msg->id == 0x600)
    {
        std::stringstream info_600;
        info_600 << "N of Clusters Far  : " << cluster_status.NofClustersFar << std::endl;
        info_600 << "N of Clusters Near : " << cluster_status.NofClustersNear << std::endl;
        info_600 << "Meas Counter       : " << cluster_status.MeasCounter << std::endl;
        info_600 << "Interface Version  : " << cluster_status.InterfaceVersion << std::endl;

        std_msgs::String str_600;
        str_600.data = info_600.str();
        pubs["cluster_status"].publish(str_600);

        // Show on rviz.
        ars408_msg::RadarPoints rps;        
        for (auto it = clusters_map.begin(); it != clusters_map.end(); it++) {
            ars408_msg::RadarPoint rp;
            
            rp.id = it->second.id;
            rp.dynProp = it->second.DynProp;
            rp.distX = it->second.DistLong;
            rp.distY = it->second.DistLat;
            rp.vrelX = it->second.VrelLong;
            rp.vrelY = it->second.VrelLat;
            rp.rcs = it->second.RCS;
            rp.strs = "This is cluster.";
            rp.prob = 0x7; // "<=100%"
            rp.classT = 0;
            rp.angle = 0;
            rp.height = 0.5;
            rp.width = 0.5;
            
            // effective range
            bool effectiveRange = true;
            #ifdef EFFECTIVE_RANGE
            effectiveRange = false;
            float angle = atan2(it->second.DistLat, it->second.DistLong) * 180 / M_PI;
            float dist = sqrt(pow(it->second.DistLong, 2) + pow(it->second.DistLat, 2));
            if((dist < (10 / cos(60 * M_PI / 180)) && abs(angle) < 60) || (dist < 70 && abs(angle) < 40) || (dist < 150 && abs(angle) < 9) || (dist < 250 && abs(angle) < 4))
            {
                effectiveRange = true;
            }
            else if(it->second.DistLong > 10 && it->second.DistLong < 70 * cos(40 * M_PI / 180))
            {
                float deltaX = (it->second.DistLong - 10) / (70 * cos(40 * M_PI / 180) - 10);
                float deltaY = deltaX * (70 * sin(40 * M_PI / 180) - (10 / cos(60 * M_PI / 180) * sin(60 * M_PI / 180))) + (10 / cos(60 * M_PI / 180) * sin(60 * M_PI / 180));
                if(abs(it->second.DistLat) < deltaY)
                    effectiveRange = true;
            }
            else if(it->second.DistLong > 150 && it->second.DistLong < 250)
            {
                float deltaX = (it->second.DistLong - 150) / (250 - 150);
                float deltaY = deltaX * ((250 / cos(4 * M_PI / 180) * sin(4 * M_PI / 180)) - (150 / cos(9 * M_PI / 180) * sin(9 * M_PI / 180))) + (150 / cos(9 * M_PI / 180) * sin(9 * M_PI / 180));
                if(abs(it->second.DistLat) < deltaY)
                    effectiveRange = true;
            }
            #endif

            if (it->second.id != -1 && effectiveRange)
            {
                std_msgs::Header header;
                header.stamp = ros::Time::now();
                header.seq = it->second.id;
                rp.header = header;
                rps.rps.push_back(rp);
            }
        }

        std_msgs::Header header;
        header.stamp = ros::Time::now();
        header.seq = this->seq;
        this->seq += 1;
        rps.header = header;
        pubs["decoded_messages"].publish(rps);
        clusters_map.clear();

        // Get New Clusters
        cluster_status.NofClustersNear = msg->data[0];
        cluster_status.NofClustersFar  = msg->data[1];

        unsigned int measCounter_raw = (msg->data[2] << 8) | (msg->data[3]);
        cluster_status.MeasCounter = measCounter_raw;

        unsigned int interfaceVersion_raw = msg->data[4] >> 4;
        cluster_status.InterfaceVersion = interfaceVersion_raw;
    }
    else if (msg->id == 0x701)
    {
        ARS408::Cluster clu;

        clu.id = msg->data[0];

        unsigned int distLong_raw = (msg->data[1] << 5) | (msg->data[2] >> 3);
        clu.DistLong = -500.0 + distLong_raw * 0.2;

        unsigned int distLat_raw = ((msg->data[2] & 0b00000011) << 8) | msg->data[3];
        clu.DistLat = -102.3 + distLat_raw * 0.2;

        unsigned int vrelLong_raw = (msg->data[4] << 2) | (msg->data[5] >> 6);
        clu.VrelLong = -128.0 + vrelLong_raw * 0.25;

        unsigned int vrelLat_raw = ((msg->data[5] & 0b00111111) << 3) | (msg->data[6] >> 5);
        clu.VrelLat = -64.0 + vrelLat_raw * 0.25;

        clu.DynProp = msg->data[6] & 0b00000111;

        unsigned int rcs_raw = msg->data[7];
        clu.RCS = -64.0 + rcs_raw * 0.5;

        clusters_map[clu.id] = clu;
    }
    else if(msg->id == 0x702){
        ARS408::Cluster::Cluster_quality clu_q;

        clu_q.id = (unsigned int)(msg->data[0]);
        clu_q.DistLong_rms = (unsigned int)(msg->data[1] >> 3);
        clu_q.DistLat_rms = (unsigned int)(msg->data[1] & 0b00000111) << 2 | (msg->data[1] >> 6);
        clu_q.VrelLong_rms = (unsigned int)(msg->data[2] & 0b00111110) >> 2;
        clu_q.VrelLat_rms = (unsigned int)(msg->data[2] & 0b00000001) << 4 | (msg->data[3] >> 4);
        clu_q.Pdh0 = (unsigned int)(msg->data[3] & 0b000000111);
        clu_q.InvalidState = (unsigned int)(msg->data[4] >> 3);
        clu_q.AmbigState = (unsigned int)(msg->data[4] & 0b00000111);

        clusters_map[clu_q.id].cluster_quality = clu_q;
    }
    #pragma endregion

    #pragma region Object
    if (msg->id == 0x60A)
    {
        std::stringstream info_60A;
        info_60A << "N of Objects      : " << object_status.NofOBjects << std::endl;
        info_60A << "Meas Counter      : " << object_status.MeasCounter << std::endl;
        info_60A << "Interface Version : " << object_status.InterfaceVersion << std::endl;

        std_msgs::String str_60A;
        str_60A.data = info_60A.str();
        pubs["object_status"].publish(str_60A);

        // Show on rviz.
        ars408_msg::RadarPoints rps;

        for (auto it = objects_map.begin(); it != objects_map.end(); ++it)
        {
            ars408_msg::RadarPoint rp;
            rp.id = it->second.id;
            rp.dynProp = it->second.DynProp;
            rp.distX = it->second.DistLong;
            rp.distY = it->second.DistLat;
            rp.vrelX = it->second.VrelLong;
            rp.vrelY = it->second.VrelLat;
            rp.rcs = it->second.RCS;
            rp.strs = "This is object.";
            rp.classT  = it->second.object_extended.Class;
            // rp.angle = it->second.object_extended.OrientationAngle;
            rp.angle = atan2(rp.distY, rp.distX);
            rp.height = it->second.object_extended.Length;
            rp.width = it->second.object_extended.Width;

            if (it->second.object_quality.id != -1)
            {
                // rp.prob = ARS408::ProbOfExist[it->second.object_quality.ProbOfExist];
                rp.prob = it->second.object_quality.ProbOfExist;
            }

            // effective range
            bool effectiveRange = true;
            #ifdef EFFECTIVE_RANGE
            effectiveRange = false;
            float angle = atan2(it->second.DistLat, it->second.DistLong) * 180 / M_PI;
            float dist = sqrt(pow(it->second.DistLong, 2) + pow(it->second.DistLat, 2));
            if((dist < (10 / cos(60 * M_PI / 180)) && abs(angle) < 60) || (dist < 70 && abs(angle) < 40) || (dist < 150 && abs(angle) < 9) || (dist < 250 && abs(angle) < 4))
            {
                effectiveRange = true;
            }
            else if(it->second.DistLong > 10 && it->second.DistLong < 70 * cos(40 * M_PI / 180))
            {
                float deltaX = (it->second.DistLong - 10) / (70 * cos(40 * M_PI / 180) - 10);
                float deltaY = deltaX * (70 * sin(40 * M_PI / 180) - (10 / cos(60 * M_PI / 180) * sin(60 * M_PI / 180))) + (10 / cos(60 * M_PI / 180) * sin(60 * M_PI / 180));
                if(abs(it->second.DistLat) < deltaY)
                    effectiveRange = true;
            }
            else if(it->second.DistLong > 150 && it->second.DistLong < 250)
            {
                float deltaX = (it->second.DistLong - 150) / (250 - 150);
                float deltaY = deltaX * ((250 / cos(4 * M_PI / 180) * sin(4 * M_PI / 180)) - (150 / cos(9 * M_PI / 180) * sin(9 * M_PI / 180))) + (150 / cos(9 * M_PI / 180) * sin(9 * M_PI / 180));
                if(abs(it->second.DistLat) < deltaY)
                    effectiveRange = true;
            }
            #endif

            if (it->second.id != -1 && effectiveRange)
            {
                std_msgs::Header header;
                header.stamp = ros::Time::now();
                header.seq = it->second.id;
                rp.header = header;
                rps.rps.push_back(rp);
            }
        }

        // ROS_INFO_STREAM(radar_name << " publishing " << rps.rps.size());

        std_msgs::Header header;
        header.stamp = ros::Time::now();
        header.seq = this->seq;
        this->seq += 1;
        rps.header = header;
        pubs["decoded_messages"].publish(rps);
        objects_map.clear();

        // Get New Objects
        object_status.NofOBjects = msg->data[0];

        unsigned int measCounter_raw = (msg->data[1] << 8) | (msg->data[2]);
        object_status.MeasCounter = measCounter_raw;

        unsigned int interfaceVersion_raw = msg->data[3] >> 4;
        object_status.InterfaceVersion = interfaceVersion_raw;
    }
    else if (msg->id == 0x60B)
    {
        ARS408::Object obj;

        obj.id = msg->data[0];

        unsigned int distLong_raw = (msg->data[1] << 5) | (msg->data[2] >> 3);
        obj.DistLong = -500.0 + distLong_raw * 0.2;

        unsigned int distLat_raw = ((msg->data[2] & 0b00000111) << 8) | (msg->data[3]);
        obj.DistLat  = -204.6 + distLat_raw * 0.2;

        unsigned int vrelLong_raw = (msg->data[4] << 2) | (msg->data[5] >> 6);
        obj.VrelLong = -128.0 + vrelLong_raw * 0.25;

        unsigned int vrelLat_raw = ((msg->data[5] & 0b00111111) << 3) | (msg->data[6] >> 5);
        obj.VrelLat  = -64.0 + vrelLat_raw * 0.25;

        obj.DynProp  = msg->data[6] & 0b00000111;

        unsigned int rcs_raw = msg->data[7];
        obj.RCS = -64.0 + rcs_raw * 0.5;

        objects_map[obj.id] = obj;
    }
    else if (msg->id == 0x60C)
    {
        ARS408::Object::Object_quality obj_q;

        obj_q.id = (unsigned int)msg->data[0];

        obj_q.DistLong_rms = msg->data[1] >> 3;
        obj_q.DistLat_rms  = (((msg->data[1] & 0b00000111) << 2) | (msg->data[2] >> 6));
        obj_q.VrelLong_rms = ((msg->data[2] & 0b00111110) >> 1);
        obj_q.VrelLat_rms  = (((msg->data[2] & 0b00000001) << 4) | msg->data[3] >> 4);
        obj_q.ArelLong_rms = ((msg->data[3] << 4) | (msg->data[4] >> 7));
        obj_q.ArelLat_rms  = ((msg->data[4] & 0b01111100) >> 2);
        obj_q.Orientation_rms = (((msg->data[4] & 0b00000011) << 3) | (msg->data[5] >> 5));
        obj_q.ProbOfExist  = ((msg->data[6] & 0b11100000) >> 5);
        obj_q.MeasState    = ((msg->data[6] & 0b00011100) >> 2);
        objects_map[obj_q.id].object_quality = obj_q;
    }
    else if (msg->id == 0x60D)
    {
        ARS408::Object::Object_extended obj_e;

        obj_e.id = msg->data[0];

        unsigned int ArelLong_raw = (msg->data[1] << 3) | (msg->data[2] >> 5);
        obj_e.ArelLong = -10.0 + ArelLong_raw * 0.01;

        unsigned int ArellLat_raw = ((msg->data[2] & 0b00011111) << 4) | (msg->data[3] >> 4);
        obj_e.ArellLat = -2.5 + ArellLat_raw * 0.01;

        obj_e.Class = msg->data[3] & 0b00000111;

        unsigned int OrientationAngle_raw = (msg->data[4] << 2) | (msg->data[5] >> 6);
        obj_e.OrientationAngle = -180.0 + OrientationAngle_raw * 0.4;

        obj_e.Length = msg->data[6] * 0.2;

        obj_e.Width = msg->data[7] * 0.2;

        objects_map[obj_e.id].object_extended = obj_e;
    }
    else if (msg->id == 0x60E)
    {
        ARS408::Object::Object_collision obj_c;

        obj_c.id = msg->data[0];

        obj_c.CollDetRegionBitfield = msg->data[1];

        objects_map[obj_c.id].object_collision = obj_c;
    }
    #pragma endregion

    #pragma region Filter
    if(msg->id == 0x203)
    {
        ARS408::FilterHeader fil_h;
        fil_h.NofClusterFilterCfg = msg->data[0] >> 3;
        fil_h.NofObjectFilterCfg  = msg->data[1] >> 3;

        #ifdef PRINT_FILTER_CONFIG
        std::stringstream info_203;
        info_203 << "Cluster Filter: " << fil_h.NofClusterFilterCfg << std::endl;
        info_203 << "Object Filter: " << fil_h.NofObjectFilterCfg << std::endl;
        std::cout << info_203.str();
        #endif
    }
    else if (msg->id == 0x204)
    {
        ARS408::FilterCfg fil_c;
        fil_c.Type  = msg->data[0] >> 7;
        fil_c.Index = (msg->data[0] & 0b01111000) >> 3;
        fil_c.Min_Distance = (msg->data[1] << 8) | msg->data[2];
        fil_c.Max_Distance = (msg->data[3] << 8) | msg->data[4];

        #ifdef PRINT_FILTER_CONFIG
        std::stringstream info_204;
        info_204 << "Filter Type: " << fil_c.Type << std::endl;
        info_204 << "Index: " << fil_c.Index << std::endl;
        info_204 << "Min Distance: " << fil_c.Min_Distance << std::endl;
        info_204 << "Max Distance: " << fil_c.Max_Distance << std::endl;
        std::cout << info_204.str();
        #endif
    }
    #pragma endregion

    #pragma region Collision
    if(msg->id == 0x408){
        ARS408::CollDetState col;

        col.NofRegions = msg->data[0] >> 4;
        col.Activation = (msg->data[0] & 0b00000010) >> 1;

        unsigned int minDetectTime_raw = msg->data[1];
        col.MinDetectTime = minDetectTime_raw * 0.1;
        col.MeasCounter = (msg->data[2] << 8) | msg->data[3];
    }
    else if(msg->id == 0x402)
    {
        ARS408::CollDetRegion col_d;

        col_d.RegionID = (msg->data[0] >> 5);
        col_d.WarningLevel = (msg->data[0] & 0b00011000) >> 3;

        unsigned int point1_x = (msg->data[1] << 5) | (msg->data[2] >> 3);
        col_d.Point1X = -500.0 + point1_x * 0.2;

        unsigned int point1_y = (msg->data[2] << 8) | msg->data[3];
        col_d.Point1Y = -204.6 + point1_y * 0.2;

        unsigned int point2_x = (msg->data[4] << 5) | (msg->data[5] >> 3);
        col_d.Point2X = -500.0 + point2_x * 0.2;

        unsigned int point2_y = (msg->data[5] << 8) | msg->data[6];
        col_d.Point2Y = -204.6 + point2_y * 0.2;

        col_d.NofObjects = msg->data[7];
    }
    #pragma endregion
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "Decode Radar");

    std::string radar_name;
    if (argc == 2) {
        radar_name = std::string(argv[1]);
        ROS_INFO_STREAM("getting radar name: " << radar_name);
    } else {
        ROS_ERROR("error getting radar name");
        return 1;
    }

    RadarDecoder radarDecoder(radar_name);
    
    ros::Rate rate(60);
    while (ros::ok()) {
	    ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
