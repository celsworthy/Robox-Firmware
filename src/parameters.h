/*

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "motion.h"   // for t_motion_parameters
#include "position.h" // for t_position_parameters
#include "heaters.h"  // for t_tempcon_parameters


typedef struct {
  unsigned long         version; // used to check that the values in flash were saved by the same version of firmware

  t_motion_parameters   motion;
  t_position_parameters position;

  t_tempcon_parameters  nozzle_tempcon;
  t_tempcon_parameters  bed_tempcon;
  t_tempcon_parameters  ambient_tempcon;
} parameter_struct;


void set_default_parameters(void);
void init_parameters(void);
void store_parameters(void);
void load_parameters(void);
void show_parameters(char *response);
void write_unique_id(unsigned char *id);
void get_unique_id(unsigned char *id);
void initialise_hours_counter(void);
void maintain_hours_counter(void);
void get_hours_counter(unsigned char *d);
void write_hours_counter(unsigned char *d);
unsigned char is_robox_pro(void);
