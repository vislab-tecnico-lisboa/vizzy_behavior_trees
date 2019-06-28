#ifndef COVERSIONS_BT_HPP_
#define COVERSIONS_BT_HPP_


namespace BT
{
    template <> inline geometry_msgs::PoseStamped convertFromString(StringView str)
    {

        // We expect real numbers separated by semicolons
        auto parts = splitString(str, ';');
        if (parts.size() != 7)
        {
            throw RuntimeError("Invalid input. It should be: x;y;z;qx;qy;qz;qw");
        }
        else{
            geometry_msgs::PoseStamped output;
            output.pose.position.x =  convertFromString<double>(parts[0]);
            output.pose.position.y =  convertFromString<double>(parts[1]);
            output.pose.position.z =  convertFromString<double>(parts[2]);
            output.pose.orientation.x =  convertFromString<double>(parts[3]);
            output.pose.orientation.y =  convertFromString<double>(parts[4]);
            output.pose.orientation.z =  convertFromString<double>(parts[5]);
            output.pose.orientation.w =  convertFromString<double>(parts[6]);

            return output;
        }
    }
} // end namespace BT

#endif

