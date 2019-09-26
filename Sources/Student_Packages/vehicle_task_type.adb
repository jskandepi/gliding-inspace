with Ada.Real_Time;              use Ada.Real_Time;
with Ada.Text_IO;                use Ada.Text_IO;
with Exceptions;                 use Exceptions;
--  with Real_Type;                  use Real_Type;
--  with Generic_Sliding_Statistics;
--  with Rotations;                  use Rotations;
with Vectors_3D;                 use Vectors_3D;
with Vehicle_Interface;          use Vehicle_Interface;
with Vehicle_Message_Type;       use Vehicle_Message_Type;
with Ada.Containers.Hashed_Maps;
with Swarm_Size;                 use Swarm_Size;
with Swarm_Structures_Base;      use Swarm_Structures_Base;
-- with Swarm_Configuration;        use Swarm_Configuration;
-- with Swarm_Configurations;       use Swarm_Configurations;
--  with Ada.Text_IO; use Ada.Text_IO;

package body Vehicle_Task_Type is

   task body Vehicle_Task is

      Vehicle_No : Positive; -- Vehicle_No of corresponding drone -- pragma Unreferenced (Vehicle_No);
      Charge_Case_1 : Boolean := False; -- Decide whether the drone needs charge. if so, then the drone goes to comparison.
      Charge_Case_2 : Boolean := False; -- emergency mode. If True, the drone head to globe directly.
      Find_Energy : Boolean := False; -- If the drone finds the globe, the parameter is set to true.
      Find_Energy_Time : Time; -- Record time when drone finds globe.
      Global_Position : Vector_3D; -- Record the position of energy globe
      Empty_Message_Flag : Boolean := True; --  if the drone hasn't received any messages then True.
      Message_Sent : Inter_Vehicle_Messages; -- The message that will be sent by the drone
      Message_Received : Inter_Vehicle_Messages; -- The message received by the drone (not useful if old)
      Message_Accepted : Inter_Vehicle_Messages; -- message accepted by drone , lastest message - contains more accurate position of globe.

      -- Record the vehicle_No and current energy level of other drones.
      package Hashtable is new Ada.Containers.Hashed_Maps (Key_Type => Positive,
                                                         Element_Type => Vehicle_Charges,
                                                         Hash => ID_Hashed,
                                                         Equivalent_Keys => "=");
      use Hashtable;
      Charge_Map : Hashtable.Map;

      -- Record the vehicle_No of live and dead vehicles.
      Vehicle_No_Set,
      Delete_No_Set : No_set;

      -- Return the minimum energy level of all the drones
      function Least_Charge return Vehicle_Charges is
         min : Vehicle_Charges := Full_Charge;
      begin
         for i in Charge_Map.Iterate loop
            if Element (i) <= min then
               min :=  Element (i);
               -- Put (min'Image);
               -- Put_Line ("flag 1");
            end if;
         end loop;
         -- Put_Line ("flag 2");
         return min;
      end Least_Charge;

   begin
      -- You need to react to this call and provide your task_id.
      -- You can e.g. employ the assigned vehicle number (Vehicle_No)
      -- in communications with other vehicles.
      accept Identify (Set_Vehicle_No : Positive; Local_Task_Id : out Task_Id) do
         Vehicle_No      := Set_Vehicle_No;
         Local_Task_Id   := Current_Task;
      end Identify;
      -- Without control this vehicle will go for its natural swarming instinct.

      select

         Flight_Termination.Stop;

      then abort
         Outer_task_loop : loop
            declare
               Globes : constant Energy_Globes := Energy_Globes_Around;
            begin
               Wait_For_Next_Physics_Update;

               -- Define at what level of current charge, the drone is in emergency mode
               -- and normal mode (charge required but not an emergency mode) .
               if Current_Charge < 0.85 then
                  if Current_Charge <= 0.5 then
                     Charge_Case_2 := True;
                  else  Charge_Case_1 := True;
                  end if;
               end if;

               -- Record the vehicle_No of this drone, which represents the drone is still alive
               if not Vehicle_No_Set.Contains (Vehicle_No) then
                  Vehicle_No_Set.Insert (Vehicle_No);
               end if;

               -- Record the vehicle_No and current_charge level of this drone
               if Charge_Map.Contains (Vehicle_No) then
                  Charge_Map.Replace (Vehicle_No, Current_Charge);
               else Charge_Map.Insert (Vehicle_No, Current_Charge);
               end if;

               -- Message received part
               while Messages_Waiting loop
                  Receive (Message_Received); -- Receive the message
                  -- Record the id and current energy level of other drones
                  if Charge_Map.Contains (Message_Received.ID) then
                     Charge_Map.Replace (Message_Received.ID, Message_Received.My_Energy);
                  else Charge_Map.Insert (Message_Received.ID, Message_Received.My_Energy);
                  end if;

                  -- Update No_Set.
                  Vehicle_No_Set.Union (Message_Received.Exist_Neighbours_No);
                  Delete_No_Set.Union (Message_Received.Delete_Neighbours_No);

                  -- if the drone has not received any messgae, receive any message even if the message is outdated.
                  if Empty_Message_Flag then
                     Empty_Message_Flag := False; -- received message , then message flag not empty.
                     Message_Accepted := Message_Received;
                     Global_Position := Message_Accepted.Energy_Globe_Pos;-- Obtain the position of globe
                  elsif not Empty_Message_Flag then
                     -- if the drone has received a message, compare the time of new message received message,
                     -- accept the new message if it is the latest one.
                     if Message_Accepted.Message_Send_Time <= Message_Received.Message_Send_Time then
                        Message_Accepted := Message_Received;
                        Global_Position := Message_Accepted.Energy_Globe_Pos;
                     end if;
                  end if;
               end loop;

               -- Update the live drones set by deleting the dead drones' number.
               Vehicle_No_Set.Difference (Delete_No_Set);

               -- globe selection
               -- Swarm_Configuration.Configuration = Single_Globe_In_Orbit
               if Globes'Length = 1 then
                  -- If there is a globe around, record the position and time when drone finds the globe.
                  Global_Position := Globes (1).Position;
                  Find_Energy_Time := Clock;
                  Find_Energy := True;
               end if;
               -- for Swarm_Configuration.Configuration = Random_Globes_In_Orbits
               -- Part c implementation
               if Globes'Length > 1  then
                  Global_Position := Globes (1).Position;
                  for Globe of Globes loop
                        if abs (Globe.Position - Position) < abs (Global_Position - Position) then
                           Global_Position := Globe.Position;
                        end if;
                  end loop;
                  Find_Energy_Time := Clock;
                  Find_Energy := True;
               end if;

               -- we will set this flag to False as  data is no longer reliable
               if Find_Energy and then (Clock - Find_Energy_Time) >= Seconds (4) then
                  Find_Energy := False;
               end if;

               -- Message sent
               Message_Sent := (ID                   => Vehicle_No,
                                Energy_Globe_Pos     => Global_Position,
                                Message_Send_Time    => Clock,
                                Energy_Globe_Find    => Find_Energy,
                                My_Energy            => Current_Charge,
                                Exist_Neighbours_No  => Vehicle_No_Set,
                                Delete_Neighbours_No => Delete_No_Set);
               Send (Message_Sent);

               -- Part D , delete the vehicle_No of extra drones if the current vehicle
               -- number is greater than Target_No_of_Elements
               if Vehicle_No > Target_No_of_Elements then
                  Delete_No_Set.Insert (Vehicle_No);
                  Put(Vehicle_No'Image);
                  exit Outer_task_loop; -- die / kill
               end if;

               -- if the drone is in emergency mode, it heads to the globe immediately.
               if Charge_Case_2 then
                  Set_Destination (Global_Position);
                  Set_Throttle (1.0);
               elsif (not Charge_Case_2 and then Charge_Case_1 and then Find_Energy)
                 or else (not Charge_Case_2 and then Charge_Case_1 and then (not Find_Energy) and then Message_Accepted.Energy_Globe_Find)
                   -- if the drone needs charge but not in emergency and it finds the position of globe OR
                   -- receive position from other drones -> comparison.
               then
                  -- If its current charge is the lowest one among all the drones, the drone heads to globe.
                  if Current_Charge <= Least_Charge then
                     Set_Destination (Global_Position);
                     Set_Throttle (0.8);
                  else
                     -- otherwise, waiting for next round comparison
                     Set_Destination (Global_Position);
                     Set_Throttle (0.2);
                  end if;
               end if;

               -- Update the information after the drone gets charged
               if Current_Charge = 1.0 then
                  Charge_Case_1  := False;
                  Charge_Case_2  := False;
                  Set_Throttle (0.2);
                  Send (Message_Sent);
                  -- Put(Message_Sent.ID'Image);
               end if;
               -- Clear the drone's charge list to get everything updated.
               Charge_Map.Clear;

            end;

         end loop Outer_task_loop;

      end select;

   exception
      when E : others => Show_Exception (E);

   end Vehicle_Task;

end Vehicle_Task_Type;
