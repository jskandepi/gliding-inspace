with Ada.Task_Identification; use Ada.Task_Identification;
with Ada.Containers.Hashed_Sets; use Ada.Containers;

package Vehicle_Task_Type is

   task type Vehicle_Task is
      entry Identify (Set_Vehicle_No : Positive; Local_Task_Id : out Task_Id);
   end Vehicle_Task;

   Function ID_Hashed (id: Positive)  return Hash_Type is (Hash_Type(id));

   package My_Set is new Ada.Containers.Hashed_Sets (Element_type => Positive,
                                                     Hash => ID_Hashed,
                                                     Equivalent_Elements => "=");
   use My_Set;

   subtype No_set is My_Set.Set;

end Vehicle_Task_Type;
