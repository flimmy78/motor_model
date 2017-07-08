// This file is released under the 3-clause BSD license. See COPYING-BSD.
// Generated by builder.sce : Please, do not edit this file
// ----------------------------------------------------------------------------
//

cur_path = get_absolute_file_path('loader.sce');
cd(cur_path);

// ulink previous function with same name
[bOK, ilib] = c_link('pi_reg_speed');
if bOK then
  ulink(ilib);
end

[bOK, ilib] = c_link('pi_reg_cur');
if bOK then
  ulink(ilib);
end
//
link('libregcur.so', 'pi_reg_cur', 'c');
link('libregspeed.so', 'pi_reg_speed', 'c');

// remove temp. variables on stack
clear my_path;
clear bOK;
clear ilib;
// ----------------------------------------------------------------------------
//pi_reg_cur_path = get_absolute_file_path('loader.sce');
