/* Copyright 2015-2017 Jack Humbert
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "planck.h"
#include "action_layer.h"
#include "muse.h"

extern keymap_config_t keymap_config;

enum planck_layers {
  _BASE,
  _LOWER,
  _RAISE,
  _LGREEK,
  _UGREEK,
  _MOUSE,
  _ADJUST
};

enum planck_keycodes {
  BASE = SAFE_RANGE,
};

enum unicodemap_index {
    // Lowercase greek
    L_ALPHA = 0,    L_BETA,     L_GAMMA,    L_DELTA,    L_EPSILON,  L_ZETA, 
    L_ETA,          L_THETA,    L_IOTA,     L_KAPPA,    L_LAMBDA,   L_MU, 
    L_NU,           L_XI,       L_OMICRON,  L_PI,       L_RHO,      L_SIGMA, 
    L_TAU,          L_UPSILON,  L_PHI,      L_CHI,      L_PSI,      L_OMEGA,

    // Uppercase greek
    U_ALPHA,        U_BETA,     U_GAMMA,    U_DELTA,    U_EPSILON,  U_ZETA, 
    U_ETA,          U_THETA,    U_IOTA,     U_KAPPA,    U_LAMBDA,   U_MU, 
    U_NU,           U_XI,       U_OMICRON,  U_PI,       U_RHO,      U_SIGMA, 
    U_TAU,          U_UPSILON,  U_PHI,      U_CHI,      U_PSI,      U_OMEGA,

    // Emoji
    THINK,
};

// Hold for layer
#define LOWER MO(_LOWER)
#define RAISE MO(_RAISE)
#define LGREK MO(_LGREEK)
#define UGREK MO(_UGREEK)

const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {

/* Qwerty
 * ,-----------------------------------------------------------------------------------.
 * | Tab  |   Q  |   W  |   E  |   R  |   T  |   Y  |   U  |   I  |   O  |   P  | Bksp |
 * |------+------+------+------+------+------+------+------+------+------+------+------|
 * | Esc  |   A  |   S  |   D  |   F  |   G  |   H  |   J  |   K  |   L  |   ;  |Enter |
 * |------+------+------+------+------+------+------+------+------+------+------+------|
 * | Shift|   Z  |   X  |   C  |   V  |   B  |   N  |   M  |   ,  |   .  |   /  |SFT(")|
 * |------+------+------+------+------+------+------+------+------+------+------+------|
 * | Ctrl | Caps | LGUI | Alt  |Lower |    Space    |Raise | Alt  | RGUI |   \  |   `  |
 * `-----------------------------------------------------------------------------------'
 */
[_BASE] = LAYOUT_planck_mit(
    KC_TAB,     KC_Q,       KC_W,       KC_E,       KC_R,       KC_T,       KC_Y,       KC_U,       KC_I,       KC_O,       KC_P,       KC_BSPC,
    KC_ESC,     KC_A,       KC_S,       KC_D,       KC_F,       KC_G,       KC_H,       KC_J,       KC_K,       KC_L,       KC_SCLN,    KC_ENT,
    KC_LSFT,    KC_Z,       KC_X,       KC_C,       KC_V,       KC_B,       KC_N,       KC_M,       KC_COMM,    KC_DOT,     KC_SLSH,    RSFT_T(KC_QUOT),
    KC_LCTL,    KC_CAPS,    KC_LGUI,    KC_LALT,    LOWER,             KC_SPC,          RAISE,      KC_RALT,    KC_LGUI,    KC_BSLS,    KC_GRV
),

/* Lower
 * ,-----------------------------------------------------------------------------------.
 * |  F1  |  F2  |  F3  |  F4  |  F5  |  F6  |  F7  |  F8  |  F9  |  F10 |  F11 | F12  |
 * |------+------+------+------+------+------+------+------+------+------+------+------|
 * |  Del |   1  |   2  |   3  |   4  |   5  |      | Prev | >/|| | Next |  -   |  =   |
 * |------+------+------+------+------+------+------+------+------+------+------+------|
 * |      |   6  |   7  |   8  |   9  |   0  |      | Vol- | Vol+ | Mute |  [   |  ]   |
 * |------+------+------+------+------+------+------+------+------+------+------+------|
 * |      |      |      |      |      |Toggle Mouse |      |      |      |      |      |
 * `-----------------------------------------------------------------------------------'
 */
[_LOWER] = LAYOUT_planck_mit(
    KC_F1,      KC_F2,      KC_F3,      KC_F4,      KC_F5,      KC_F6,      KC_F7,      KC_F8,      KC_F9,      KC_F10,     KC_F11,     KC_F12,
    KC_DEL,     KC_1,       KC_2,       KC_3,       KC_4,       KC_5,       _______,    KC_MPRV,    KC_MPLY,    KC_MNXT,    KC_MINS,    KC_EQL,
    _______,    KC_6,       KC_7,       KC_8,       KC_9,       KC_0,       _______,    KC_VOLD,    KC_VOLU,    KC_MUTE,    KC_LBRC,    KC_RBRC,
    _______,    _______,    _______,    _______,    _______,        TO(_LGREEK),         _______,    _______,    _______,    _______,    _______
),

/* Raise
 * ,-----------------------------------------------------------------------------------.
 * |  F1  |  F2  |  F3  |  F4  |  F5  |  F6  |  F7  |  F8  |  F9  |  F10 |  F11 | F12  |
 * |------+------+------+------+------+------+------+------+------+------+------+------|
 * |  Ins |   !  |   @  |   #  |   $  |   %  | Left | Down |  Up  |Right |  _   |  +   |
 * |------+------+------+------+------+------+------+------+------+------+------+------|
 * |      |   ^  |   &  |   *  |   (  |   )  | Home | PGUP | PGDN | End  |  {   |  }   |
 * |------+------+------+------+------+------+------+------+------+------+------+------|
 * |      |      |      |      |      |Toggle Mouse |      |      |      |      |PNTSCR|
 * `-----------------------------------------------------------------------------------'
 */
[_RAISE] = LAYOUT_planck_mit(
    KC_F1,      KC_F2,      KC_F3,      KC_F4,      KC_F5,      KC_F6,      KC_F7,      KC_F8,      KC_F9,      KC_F10,     KC_F11,     KC_F12,
    KC_INS,     KC_EXLM,    KC_AT,      KC_HASH,    KC_DLR,     KC_PERC,    KC_LEFT,    KC_DOWN,    KC_UP,      KC_RIGHT,   KC_UNDS,    KC_PLUS, 
    _______,    KC_CIRC,    KC_AMPR,    KC_ASTR,    KC_LPRN,    KC_RPRN,    KC_HOME,    KC_PGDN,    KC_PGUP,    KC_END,     KC_LCBR,    KC_RCBR,
    _______,    _______,    _______,    _______,    _______,        TO(_LGREEK),         _______,    _______,    _______,    _______,    KC_PSCR
),

/* Lower Greek
 * ,-----------------------------------------------------------------------------------.
 * |      |      |      |      |      |      |      |  μ   |      |      |      |      |
 * |------+------+------+------+------+------+------+------+------+------+------+------|
 * |      |      |      |  Δ   |      |      |      |      |      |      |      |      |
 * |------+------+------+------+------+------+------+------+------+------+------+------|
 * |UGREK |      |      |      |      |      |      |      |      |      |      |UGREK |
 * |------+------+------+------+------+------+------+------+------+------+------+------|
 * |      |      |      |      | Base |             | Base |      |      |      |      |
 * `-----------------------------------------------------------------------------------'
 */
[_LGREEK] = LAYOUT_planck_mit(
    _______,    _______,    X(L_OMEGA), X(L_EPSILON),   X(L_RHO),   X(L_TAU),   X(L_THETA), X(L_UPSILON),   X(L_IOTA),      X(L_OMICRON),   X(L_PI),     _______,
    _______,    X(L_ALPHA), X(L_SIGMA), X(L_DELTA),     _______,    X(L_GAMMA), X(L_ETA),   _______,        X(L_KAPPA),     X(L_LAMBDA),    X(L_PHI),    _______,
    UGREK,      X(L_ZETA),  X(L_XI),    X(L_CHI),       _______,    X(L_BETA),  X(L_NU),    X(L_MU),        _______,        _______,        X(L_PSI),    UGREK,
    _______,    _______,    _______,    _______,        TO(_BASE),         _______,         TO(_BASE),      _______,        _______,        _______,     _______
),

/* Upper Greek
 * ,-----------------------------------------------------------------------------------.
 * |      |      |      |      |      |      |      |  μ   |      |      |      |      |
 * |------+------+------+------+------+------+------+------+------+------+------+------|
 * |      |      |      |  Δ   |      |      |      |      |      |      |      |      |
 * |------+------+------+------+------+------+------+------+------+------+------+------|
 * |      |      |      |      |      |      |      |      |      |      |      |      |
 * |------+------+------+------+------+------+------+------+------+------+------+------|
 * |      |      |      |      | Base |             | Base |      |      |      |      |
 * `-----------------------------------------------------------------------------------'
 */

[_UGREEK] = LAYOUT_planck_mit(
    _______,    _______,    X(U_OMEGA), X(U_EPSILON),   X(U_RHO),   X(U_TAU),   X(U_THETA), X(U_UPSILON),   X(U_IOTA),      X(U_OMICRON),   X(U_PI),     _______,
    _______,    X(U_ALPHA), X(U_SIGMA), X(U_DELTA),     _______,    X(U_GAMMA), X(U_ETA),   _______,        X(U_KAPPA),     X(U_LAMBDA),    X(U_PHI),    _______,
    _______,    X(U_ZETA),  X(U_XI),    X(U_CHI),       _______,    X(U_BETA),  X(U_NU),    X(U_MU),        _______,        _______,        X(U_PSI),  _______,
    _______,    _______,    _______,    _______,        TO(_BASE),         _______,         TO(_BASE),      _______,        _______,        _______,     _______
),
/* Mouse
 * ,-----------------------------------------------------------------------------------.
 * |      |      |      |  MU  |      |      |      |      |      |      |      |      |
 * |------+------+------+------+------+------+------+------+------+------+------+------|
 * |      |      |  ML  |  MD  |  MR  |      |      |MBTN1 |MBTN2 |MBTN3 |MBTN4 |MBTN5 |
 * |------+------+------+------+------+------+------+------+------+------+------+------|
 * | ACL0 | ACL1 | ACL2 |      |      |      |      |      |      |      |      |      |
 * |------+------+------+------+------+------+------+------+------+------+------+------|
 * |      |      |      |      | Base |             | Base | WHL  | WHD  | WHU  | WHR  |
 * `-----------------------------------------------------------------------------------'
 */
[_MOUSE] = LAYOUT_planck_mit(
    _______,    _______,    _______,    KC_MS_U,    _______,    _______,    _______,   _______,     _______,    _______,   _______,     _______,
    _______,    _______,    KC_MS_L,    KC_MS_D,    KC_MS_R,    _______,    _______,   KC_BTN1,     KC_BTN2,    KC_BTN3,   KC_BTN4,     KC_BTN5,
    KC_ACL0,    KC_ACL1,    KC_ACL2,    _______,    _______,    _______,    _______,   _______,     _______,    _______,   _______,     _______,
    _______,    _______,    _______,    _______,    TO(_BASE),         _______,        TO(_BASE),   KC_WH_L,    KC_WH_D,   KC_WH_U,     KC_WH_R
),

/* Adjust (Lower + Raise)
 * ,-----------------------------------------------------------------------------------.
 * |      | Reset|      |      |      |      |      |      |      |      |      |      |
 * |------+------+------+------+------+------+------+------+------+------+------+------|
 * |      |      |      |Aud on|Audoff|AGnorm|AGswap|Qwerty|      |      |      |      |
 * |------+------+------+------+------+------+------+------+------+------+------+------|
 * |      |Voice-|Voice+|Mus on|Musoff|MIDIon|MIDIof|      |      |      |      |      |
 * |------+------+------+------+------+------+------+------+------+------+------+------|
 * |      |      |      |      |      |             |      |      |      |      |      |
 * `-----------------------------------------------------------------------------------'
 */
[_ADJUST] = LAYOUT_planck_grid(
    _______, RESET,   DEBUG,   RGB_TOG, RGB_MOD, RGB_HUI, RGB_HUD, RGB_SAI, RGB_SAD,  RGB_VAI, RGB_VAD, _______,
    _______, _______, MU_MOD,  AU_ON,   AU_OFF,  AG_NORM, AG_SWAP, BASE,    _______,  _______, _______, _______,
    _______, MUV_DE,  MUV_IN,  MU_ON,   MU_OFF,  MI_ON,   MI_OFF,  TERM_ON, TERM_OFF, _______, _______, _______,
    _______, _______, _______, _______, _______, _______, _______, _______, _______,  _______, _______, _______
)


};

uint32_t layer_state_set_user(uint32_t state) {
  return update_tri_layer_state(state, _LOWER, _RAISE, _ADJUST);
}

bool process_record_user(uint16_t keycode, keyrecord_t *record) {
  switch (keycode) {
    case BASE:
      if (record->event.pressed) {
        set_single_persistent_default_layer(_BASE);
      }
      return false;
      break;
  }
  return true;
}

bool muse_mode = false;
uint8_t last_muse_note = 0;
uint16_t muse_counter = 0;
uint8_t muse_offset = 70;
uint16_t muse_tempo = 50;

void encoder_update(bool clockwise) {
  if (muse_mode) {
    if (IS_LAYER_ON(_RAISE)) {
      if (clockwise) {
        muse_offset++;
      } else {
        muse_offset--;
      }
    } else {
      if (clockwise) {
        muse_tempo+=1;
      } else {
        muse_tempo-=1;
      }
    }
  } else {
    if (clockwise) {
      register_code(KC_PGDN);
      unregister_code(KC_PGDN);
    } else {
      register_code(KC_PGUP);
      unregister_code(KC_PGUP);
    }
  }
}

void dip_update(uint8_t index, bool active) {
  switch (index) {
    case 0:
      if (active) {
        layer_on(_ADJUST);
      } else {
        layer_off(_ADJUST);
      }
      break;
    case 1:
      if (active) {
        muse_mode = true;
      } else {
        muse_mode = false;
        #ifdef AUDIO_ENABLE
          stop_all_notes();
        #endif
      }
   }
}

void matrix_scan_user(void) {
  #ifdef AUDIO_ENABLE
    if (muse_mode) {
      if (muse_counter == 0) {
        uint8_t muse_note = muse_offset + SCALE[muse_clock_pulse()];
        if (muse_note != last_muse_note) {
          stop_note(compute_freq_for_midi_note(last_muse_note));
          play_note(compute_freq_for_midi_note(muse_note), 0xF);
          last_muse_note = muse_note;
        }
      }
      muse_counter = (muse_counter + 1) % muse_tempo;
    }
  #endif
}

bool music_mask_user(uint16_t keycode) {
  switch (keycode) {
    case RAISE:
    case LOWER:
      return false;
    default:
      return true;
  }
}

// Unicode things
void matrix_init_user(void) {
    set_unicode_input_mode(UC_WINC);
}

const uint32_t PROGMEM unicode_map[] = {
    // Note: L_RHO = 0x03C1, L_SIGMA = 0x03C3
    // Note: U_RHO = 0x03A1, U_SIGMA = 0x03A3
    // This is not a mistake, there is a special form of sigma.

    // Lowercase Greek
    0x03B1, 0x03B2, 0x03B3, 0x03B4, 0x03B5, 0x03B6, 
    0x03B7, 0x03B8, 0x03B9, 0x03BA, 0x03BB, 0x03BC, 
    0x03BD, 0x03BE, 0x03BF, 0x03C0, 0x03C1, 0x03C3, 
    0x03C4, 0x03C5, 0x03C6, 0x03C7, 0x03C8, 0x03C9,

    // Uppercase Greek
    0x0391, 0x0392, 0x0393, 0x0394, 0x0395, 0x0396,
    0x0397, 0x0398, 0x0399, 0x039A, 0x039B, 0x039C,
    0x039D, 0x039E, 0x039F, 0x03A0, 0x03A1, 0x03A3,
    0x03A4, 0x03A5, 0x03A6, 0x03A7, 0x03A8, 0x03A9,

    // Emoji
    0x0001F914
};
