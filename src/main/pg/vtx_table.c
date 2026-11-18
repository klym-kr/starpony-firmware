/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <string.h>
#include "common/printf.h"

#include "platform.h"

#ifdef USE_VTX_TABLE

#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "pg/vtx_table.h"
#include "drivers/vtx_common.h"
#include "drivers/vtx_table.h"

PG_REGISTER_WITH_RESET_FN(vtxTableConfig_t, vtxTableConfig, PG_VTX_TABLE_CONFIG, 0);

void pgResetFn_vtxTableConfig(vtxTableConfig_t *config)
{
    // Populate DEC1 Spot 2.5W 64Ch default table for St4rpony
    config->bands = 8;
    config->channels = 8;
    config->powerLevels = 5;

    // Band 1: BAND_A (A)
    static const uint16_t bandA[8] = { 5865, 5845, 5825, 5805, 5785, 5765, 5745, 5725 };
    vtxTableStrncpyWithPad(config->bandNames[0], "BAND_A", VTX_TABLE_BAND_NAME_LENGTH);
    config->bandLetters[0] = 'A';
    config->isFactoryBand[0] = false;
    memcpy(config->frequency[0], bandA, sizeof(bandA));

    // Band 2: BAND_B (B)
    static const uint16_t bandB[8] = { 5733, 5752, 5771, 5790, 5809, 5828, 5847, 5866 };
    vtxTableStrncpyWithPad(config->bandNames[1], "BAND_B", VTX_TABLE_BAND_NAME_LENGTH);
    config->bandLetters[1] = 'B';
    config->isFactoryBand[1] = false;
    memcpy(config->frequency[1], bandB, sizeof(bandB));

    // Band 3: BAND_E (E)
    static const uint16_t bandE[8] = { 5705, 5685, 5665, 5645, 5885, 5905, 5925, 5945 };
    vtxTableStrncpyWithPad(config->bandNames[2], "BAND_E", VTX_TABLE_BAND_NAME_LENGTH);
    config->bandLetters[2] = 'E';
    config->isFactoryBand[2] = false;
    memcpy(config->frequency[2], bandE, sizeof(bandE));

    // Band 4: BAND_F (F)
    static const uint16_t bandF[8] = { 5740, 5760, 5780, 5800, 5820, 5840, 5860, 5880 };
    vtxTableStrncpyWithPad(config->bandNames[3], "BAND_F", VTX_TABLE_BAND_NAME_LENGTH);
    config->bandLetters[3] = 'F';
    config->isFactoryBand[3] = false;
    memcpy(config->frequency[3], bandF, sizeof(bandF));

    // Band 5: BAND_R (R) - Primary Raceband
    static const uint16_t bandR[8] = { 5658, 5695, 5732, 5769, 5806, 5843, 5880, 5917 };
    vtxTableStrncpyWithPad(config->bandNames[4], "BAND_R", VTX_TABLE_BAND_NAME_LENGTH);
    config->bandLetters[4] = 'R';
    config->isFactoryBand[4] = false;
    memcpy(config->frequency[4], bandR, sizeof(bandR));

    // Band 6: BAND_L (L)
    static const uint16_t bandL[8] = { 5362, 5399, 5436, 5473, 5510, 5547, 5584, 5621 };
    vtxTableStrncpyWithPad(config->bandNames[5], "BAND_L", VTX_TABLE_BAND_NAME_LENGTH);
    config->bandLetters[5] = 'L';
    config->isFactoryBand[5] = false;
    memcpy(config->frequency[5], bandL, sizeof(bandL));

    // Band 7: BAND_X (X)
    static const uint16_t bandX[8] = { 4990, 5020, 5050, 5080, 5110, 5140, 5170, 5200 };
    vtxTableStrncpyWithPad(config->bandNames[6], "BAND_X", VTX_TABLE_BAND_NAME_LENGTH);
    config->bandLetters[6] = 'X';
    config->isFactoryBand[6] = false;
    memcpy(config->frequency[6], bandX, sizeof(bandX));

    // Band 8: BAND_S (S)
    static const uint16_t bandS[8] = { 6002, 6028, 6054, 6080, 6106, 6132, 6158, 6184 };
    vtxTableStrncpyWithPad(config->bandNames[7], "BAND_S", VTX_TABLE_BAND_NAME_LENGTH);
    config->bandLetters[7] = 'S';
    config->isFactoryBand[7] = false;
    memcpy(config->frequency[7], bandS, sizeof(bandS));

    // Power values: 1 (OFF), 14 (25mW), 27 (500mW), 30 (1000mW = 1W), 34 (2500mW = 2.5W)
    static const uint16_t powerValues[5] = { 1, 14, 27, 30, 34 };
    memcpy(config->powerValues, powerValues, sizeof(powerValues));

    // Power labels: OFF, 25, 500, 1, 2.5
    vtxTableStrncpyWithPad(config->powerLabels[0], "OFF", VTX_TABLE_POWER_LABEL_LENGTH);
    vtxTableStrncpyWithPad(config->powerLabels[1], "25", VTX_TABLE_POWER_LABEL_LENGTH);
    vtxTableStrncpyWithPad(config->powerLabels[2], "500", VTX_TABLE_POWER_LABEL_LENGTH);
    vtxTableStrncpyWithPad(config->powerLabels[3], "1", VTX_TABLE_POWER_LABEL_LENGTH);
    vtxTableStrncpyWithPad(config->powerLabels[4], "2.5", VTX_TABLE_POWER_LABEL_LENGTH);
}
#endif
