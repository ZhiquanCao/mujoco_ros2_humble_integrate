#include "mujoco/mujoco.h"
const double zoom_increment = 0.02;  // ratio of one click-wheel zoom increment to vertical extent



void UiModify(mjUI* ui, mjuiState* state, mjrContext* con) {
  mjui_resize(ui, con);
  mjr_addAux(ui->auxid, ui->width, ui->maxheight, ui->spacing.samples, con);
  UiLayout(state);
  mjui_update(-1, -1, ui, state, con);
}



// update an entire section of ui0
static void mjui0_update_section(mj::Simulate* sim, int section) {
  mjui_update(section, -1, &sim->ui0, &sim->uistate, &sim->platform_ui->mjr_context());
}



// handle UI event
void UiEvent(mjuiState* state) {
  mj::Simulate* sim = static_cast<mj::Simulate*>(state->userdata);

  // call UI 0 if event is directed to it
  if ((state->dragrect==sim->ui0.rectid) ||
      (state->dragrect==0 && state->mouserect==sim->ui0.rectid) ||
      state->type==mjEVENT_KEY) {
    // process UI event
    mjuiItem* it = mjui_event(&sim->ui0, state, &sim->platform_ui->mjr_context());

    // file section
    if (it && it->sectionid==SECT_FILE) {
      switch (it->itemid) {
      case 0:             // Save xml
        sim->pending_.save_xml = GetSavePath("mjmodel.xml");
        break;

      case 1:             // Save mjb
        sim->pending_.save_mjb = GetSavePath("mjmodel.mjb");
        break;

      case 2:             // Print model
        sim->pending_.print_model = GetSavePath("MJMODEL.TXT");
        break;

      case 3:             // Print data
        sim->pending_.print_data = GetSavePath("MJDATA.TXT");
        break;

      case 4:             // Quit
        sim->exitrequest.store(1);
        break;

      case 5:             // Screenshot
        sim->screenshotrequest.store(true);
        break;
      }
    }

    // option section
    else if (it && it->sectionid==SECT_OPTION) {
      if (it->pdata == &sim->spacing) {
        sim->ui0.spacing = mjui_themeSpacing(sim->spacing);
        sim->ui1.spacing = mjui_themeSpacing(sim->spacing);
      } else if (it->pdata == &sim->color) {
        sim->ui0.color = mjui_themeColor(sim->color);
        sim->ui1.color = mjui_themeColor(sim->color);
      } else if (it->pdata == &sim->font) {
        mjr_changeFont(50*(sim->font+1), &sim->platform_ui->mjr_context());
      } else if (it->pdata == &sim->fullscreen) {
        sim->platform_ui->ToggleFullscreen();
      } else if (it->pdata == &sim->vsync) {
        sim->platform_ui->SetVSync(sim->vsync);
      }

      // modify UI
      UiModify(&sim->ui0, state, &sim->platform_ui->mjr_context());
      UiModify(&sim->ui1, state, &sim->platform_ui->mjr_context());
    }

    // simulation section
    else if (it && it->sectionid==SECT_SIMULATION) {
      switch (it->itemid) {
      case 1:             // Reset
        sim->pending_.reset = true;
        break;

      case 2:             // Reload
        sim->uiloadrequest.fetch_add(1);
        break;

      case 3:             // Align
        sim->pending_.align = true;
        break;

      case 4:             // Copy pose
        sim->pending_.copy_pose = true;
        break;

      case 5:             // Adjust key
      case 6:             // Load key
        sim->pending_.load_key = true;
        break;

      case 7:             // Save key
        sim->pending_.save_key = true;
        break;

      case 11:            // History scrubber
        sim->run = 0;
        sim->pending_.load_from_history = true;
        mjui0_update_section(sim, SECT_SIMULATION);
        break;
      }
    }

    // physics section
    else if (it && it->sectionid==SECT_PHYSICS && sim->m_) {
      mjOption* opt = sim->is_passive_ ? &sim->scnstate_.model.opt : &sim->m_->opt;

      // update disable flags in mjOption
      opt->disableflags = 0;
      for (int i=0; i<mjNDISABLE; i++) {
        if (sim->disable[i]) {
          opt->disableflags |= (1<<i);
        }
      }

      // update enable flags in mjOption
      opt->enableflags = 0;
      for (int i=0; i<mjNENABLE; i++) {
        if (sim->enable[i]) {
          opt->enableflags |= (1<<i);
        }
      }

      // update disableactuator bitflag in mjOption
      bool group_changed = false;
      for (int i=0; i<mjNGROUP; i++) {
        if ((!sim->enableactuator[i]) != (opt->disableactuator & (1<<i))) {
          group_changed = true;
          if (!sim->enableactuator[i]) {
            // disable actuator group i
            opt->disableactuator |= (1<<i);
          } else {
            // enable actuator group i
            opt->disableactuator &= ~(1<<i);
          }
        }
      }

      // remake control section if actuator disable group changed
      if (group_changed) {
        sim->pending_.ui_remake_ctrl = true;
      }
    }

    // rendering section
    else if (it && it->sectionid==SECT_RENDERING) {
      // set camera in mjvCamera
      if (sim->camera==0) {
        sim->cam.type = mjCAMERA_FREE;
      } else if (sim->camera==1) {
        if (sim->pert.select>0) {
          sim->cam.type = mjCAMERA_TRACKING;
          sim->cam.trackbodyid = sim->pert.select;
          sim->cam.fixedcamid = -1;
        } else {
          sim->cam.type = mjCAMERA_FREE;
          sim->camera = 0;
          mjui0_update_section(sim, SECT_RENDERING);
        }
      } else {
        sim->cam.type = mjCAMERA_FIXED;
        sim->cam.fixedcamid = sim->camera - 2;
      }
      // copy camera spec to clipboard (as MJCF element)
      if (it->itemid == 3) {
        CopyCamera(sim);
      }
    }

    // visualization section
    else if (it && it->sectionid==SECT_VISUALIZATION) {
      if (!mju::strcmp_arr(it->name, "Align")) {
        sim->pending_.align = true;
      }
    }

    // group section
    else if (it && it->sectionid==SECT_GROUP) {
      // remake joint section if joint group changed
      if (it->name[0]=='J' && it->name[1]=='o') {
        sim->ui1.nsect = SECT_JOINT;
        MakeJointSection(sim, sim->ui1.sect[SECT_JOINT].state);
        sim->ui1.nsect = NSECT1;
        UiModify(&sim->ui1, state, &sim->platform_ui->mjr_context());
      }

      // remake control section if actuator group changed
      if (it->name[0]=='A' && it->name[1]=='c') {
        sim->pending_.ui_remake_ctrl = true;
      }
    }

    // stop if UI processed event
    if (it!=nullptr || (state->type==mjEVENT_KEY && state->key==0)) {
      return;
    }
  }

  // call UI 1 if event is directed to it
  if ((state->dragrect==sim->ui1.rectid) ||
      (state->dragrect==0 && state->mouserect==sim->ui1.rectid) ||
      state->type==mjEVENT_KEY) {
    // process UI event
    mjuiItem* it = mjui_event(&sim->ui1, state, &sim->platform_ui->mjr_context());

    // control section
    if (it && it->sectionid==SECT_CONTROL) {
      // clear controls
      if (it->itemid==0) {
        sim->pending_.zero_ctrl = true;
      }
    }

    // stop if UI processed event
    if (it!=nullptr || (state->type==mjEVENT_KEY && state->key==0)) {
      return;
    }
  }

  // shortcut not handled by UI
  if (state->type==mjEVENT_KEY && state->key!=0) {
    switch (state->key) {
    case ' ':                   // Mode
      if (!sim->is_passive_ && sim->m_) {
        sim->run = 1 - sim->run;
        sim->pert.active = 0;

        if (sim->run) sim->scrub_index = 0;  // reset scrubber

        mjui0_update_section(sim, -1);
      }
      break;

    case mjKEY_RIGHT:           // step forward
      if (!sim->is_passive_ && sim->m_ && !sim->run) {
        ClearTimers(sim->d_);

        // currently in scrubber: increment scrub, load state, update slider UI
        if (sim->scrub_index < 0) {
          sim->scrub_index++;
          sim->pending_.load_from_history = true;
          mjui0_update_section(sim, SECT_SIMULATION);
        }

        // not in scrubber: step, add to history buffer
        else {
          mj_step(sim->m_, sim->d_);
          sim->AddToHistory();
        }

        UpdateProfiler(sim, sim->m_, sim->d_);
        UpdateSensor(sim, sim->m_, sim->d_);
        UpdateSettings(sim, sim->m_);
      }
      break;

    case mjKEY_LEFT:           // step backward
      if (!sim->is_passive_ && sim->m_) {
        sim->run = 0;
        ClearTimers(sim->d_);

        // decrement scrub, load state
        sim->scrub_index = mjMAX(sim->scrub_index - 1, 1 - sim->nhistory_);
        sim->pending_.load_from_history = true;

        // update slider UI, profiler, sensor
        mjui0_update_section(sim, SECT_SIMULATION);
        UpdateProfiler(sim, sim->m_, sim->d_);
        UpdateSensor(sim, sim->m_, sim->d_);
      }
      break;

    case mjKEY_PAGE_UP:         // select parent body
      if ((sim->m_ || sim->is_passive_) && sim->pert.select > 0) {
        sim->pert.select = sim->body_parentid_[sim->pert.select];
        sim->pert.flexselect = -1;
        sim->pert.skinselect = -1;

        // stop perturbation if world reached
        if (sim->pert.select<=0) {
          sim->pert.active = 0;
        }
      }

      break;

    case ']':                   // cycle up fixed cameras
      if ((sim->m_ || !sim->is_passive_) && sim->ncam_) {
        sim->cam.type = mjCAMERA_FIXED;
        // camera = {0 or 1} are reserved for the free and tracking cameras
        if (sim->camera < 2 || sim->camera == 2 + sim->ncam_ - 1) {
          sim->camera = 2;
        } else {
          sim->camera += 1;
        }
        sim->cam.fixedcamid = sim->camera - 2;
        mjui0_update_section(sim, SECT_RENDERING);
      }
      break;

    case '[':                   // cycle down fixed cameras
      if ((sim->m_ || sim->is_passive_) && sim->ncam_) {
        sim->cam.type = mjCAMERA_FIXED;
        // camera = {0 or 1} are reserved for the free and tracking cameras
        if (sim->camera <= 2) {
          sim->camera = 2 + sim->ncam_-1;
        } else {
          sim->camera -= 1;
        }
        sim->cam.fixedcamid = sim->camera - 2;
        mjui0_update_section(sim, SECT_RENDERING);
      }
      break;

    case mjKEY_F6:                   // cycle frame visualisation
      if (sim->m_ || sim->is_passive_) {
        sim->opt.frame = (sim->opt.frame + 1) % mjNFRAME;
        mjui0_update_section(sim, SECT_RENDERING);
      }
      break;

    case mjKEY_F7:                   // cycle label visualisation
      if (sim->m_ || sim->is_passive_) {
        sim->opt.label = (sim->opt.label + 1) % mjNLABEL;
        mjui0_update_section(sim, SECT_RENDERING);
      }
      break;

    case mjKEY_ESCAPE:          // free camera
      sim->cam.type = mjCAMERA_FREE;
      sim->camera = 0;
      mjui0_update_section(sim, SECT_RENDERING);
      break;

    case '-':                   // slow down
      if (!sim->is_passive_) {
        int numclicks = sizeof(sim->percentRealTime) / sizeof(sim->percentRealTime[0]);
        if (sim->real_time_index < numclicks-1 && !state->shift) {
          sim->real_time_index++;
          sim->speed_changed = true;
        }
      }
      break;

    case '=':                   // speed up
      if (!sim->is_passive_ && sim->real_time_index > 0 && !state->shift) {
        sim->real_time_index--;
        sim->speed_changed = true;
      }
      break;

    case mjKEY_TAB:             // toggle left/right UI
      if (!state->shift) {
        // toggle left UI
        sim->ui0_enable = !sim->ui0_enable;
        UiModify(&sim->ui0, state, &sim->platform_ui->mjr_context());
      } else {
        // toggle right UI
        sim->ui1_enable = !sim->ui1_enable;
        UiModify(&sim->ui1, state, &sim->platform_ui->mjr_context());
      }
      break;
    }

    return;
  }

  // 3D scroll
  if (state->type==mjEVENT_SCROLL && state->mouserect==3) {
    // emulate vertical mouse motion = 2% of window height
    if (sim->m_ && !sim->is_passive_) {
      mjv_moveCamera(sim->m_, mjMOUSE_ZOOM, 0, -zoom_increment*state->sy, &sim->scn, &sim->cam);
    } else {
      mjv_moveCameraFromState(
          &sim->scnstate_, mjMOUSE_ZOOM, 0, -zoom_increment*state->sy, &sim->scn, &sim->cam);
    }
    return;
  }

  // 3D press
  if (state->type==mjEVENT_PRESS && state->mouserect==3) {
    // set perturbation
    int newperturb = 0;
    if (state->control && sim->pert.select>0 && (sim->m_ || sim->is_passive_)) {
      // right: translate;  left: rotate
      if (state->right) {
        newperturb = mjPERT_TRANSLATE;
      } else if (state->left) {
        newperturb = mjPERT_ROTATE;
      }
      if (newperturb && !sim->pert.active) {
        sim->pending_.newperturb = newperturb;
      }
    }

    // handle double-click
    if (state->doubleclick && (sim->m_ || sim->is_passive_)) {
      sim->pending_.select = true;
      std::memcpy(&sim->pending_.select_state, state, sizeof(sim->pending_.select_state));

      // stop perturbation on select
      sim->pert.active = 0;
      sim->pending_.newperturb = 0;
    }

    return;
  }

  // 3D release
  if (state->type==mjEVENT_RELEASE && state->dragrect==3 && (sim->m_ || sim->is_passive_)) {
    // stop perturbation
    sim->pert.active = 0;
    sim->pending_.newperturb = 0;
    return;
  }

  // 3D move
  if (state->type==mjEVENT_MOVE && state->dragrect==3 && (sim->m_ || sim->is_passive_)) {
    // determine action based on mouse button
    mjtMouse action;
    if (state->right) {
      action = state->shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    } else if (state->left) {
      action = state->shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    } else {
      action = mjMOUSE_ZOOM;
    }

    // move perturb or camera
    mjrRect r = state->rect[3];
    if (sim->pert.active) {
      if (!sim->is_passive_) {
        mjv_movePerturb(
            sim->m_, sim->d_, action, state->dx / r.height, -state->dy / r.height,
            &sim->scn, &sim->pert);
      } else {
        mjv_movePerturbFromState(
            &sim->scnstate_, action, state->dx / r.height, -state->dy / r.height,
            &sim->scn, &sim->pert);
      }
    } else {
      if (!sim->is_passive_) {
        mjv_moveCamera(
            sim->m_, action, state->dx / r.height, -state->dy / r.height,
            &sim->scn, &sim->cam);
      } else {
        mjv_moveCameraFromState(
            &sim->scnstate_, action, state->dx / r.height, -state->dy / r.height,
            &sim->scn, &sim->cam);
      }
    }
    return;
  }

  // Dropped files
  if (state->type == mjEVENT_FILESDROP && state->dropcount > 0 && !sim->is_passive_) {
    while (sim->droploadrequest.load()) {}
    mju::strcpy_arr(sim->dropfilename, state->droppaths[0]);
    sim->droploadrequest.store(true);
    return;
  }

  // Redraw
  if (state->type == mjEVENT_REDRAW) {
    sim->Render();
    return;
  }
}