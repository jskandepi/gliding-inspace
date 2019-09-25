-- Suggestions for packages which might be useful:

with Ada.Real_Time;         use Ada.Real_Time;
with Swarm_Size;            use Swarm_Size;
with Vectors_3D;            use Vectors_3D;
with Vehicle_Task_Type;     use Vehicle_Task_Type;
with Swarm_Structures_Base; use Swarm_Structures_Base;

package Vehicle_Message_Type is

   -- Replace this record definition by what your vehicles need to communicate.

   type Inter_Vehicle_Messages is record
      ID : Positive; -- For recognizing the source
      Message_Send_Time : Time;
      Energy_Globe_Find : Boolean;
      Energy_Globe_Pos : Vector_3D; -- pos to position ( weird error )
      My_Energy : Vehicle_Charges;  --  My_Energy
      Exist_Neighbours_No : No_Set; -- No_add -> check
      Delete_Neighbours_No : No_Set;
   end record;
end Vehicle_Message_Type;

