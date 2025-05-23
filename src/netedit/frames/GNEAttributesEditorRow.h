/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.dev/sumo
// Copyright (C) 2001-2025 German Aerospace Center (DLR) and others.
// This program and the accompanying materials are made available under the
// terms of the Eclipse Public License 2.0 which is available at
// https://www.eclipse.org/legal/epl-2.0/
// This Source Code may also be made available under the following Secondary
// Licenses when the conditions for such availability set forth in the Eclipse
// Public License 2.0 are satisfied: GNU General Public License, version 2
// or later which is available at
// https://www.gnu.org/licenses/old-licenses/gpl-2.0-standalone.html
// SPDX-License-Identifier: EPL-2.0 OR GPL-2.0-or-later
/****************************************************************************/
/// @file    GNEAttributesEditorRow.h
/// @author  Pablo Alvarez Lopez
/// @date    Nov 2024
///
// Row used for edit attributes in GNEAttributesEditor
/****************************************************************************/
#pragma once
#include <config.h>

#include <netedit/elements/GNEAttributeCarrier.h>

#include "GNEAttributesEditor.h"

// ===========================================================================
// class declaration
// ===========================================================================

class GNEAttributesEditor;
class GNEAttributeCarrier;

// ===========================================================================
// class definitions
// ===========================================================================

class GNEAttributesEditorRow : protected FXHorizontalFrame {
    /// @brief FOX-declaration
    FXDECLARE(GNEAttributesEditorRow)

public:
    /// @brief constructor
    GNEAttributesEditorRow(GNEAttributesEditor* attributeTable);

    /// @brief return true if attribute row was successfully show
    bool showAttributeRow(const GNEAttributeProperties* attrProperty, const bool forceDisable);

    /// @brief hide attribute row (always return false)
    bool hideAttributeRow();

    /// @brief check if current attribute row is shown
    bool isAttributeRowShown() const;

    /// @name FOX-callbacks
    /// @{

    /// @brief set new string/bool attribute
    long onCmdSetAttribute(FXObject* obj, FXSelector, void*);

    /// @brief called when user press the checkBox for toogle enable/disable attribute
    long onCmdToggleEnableAttribute(FXObject*, FXSelector, void*);

    /// @brief called when user press "edit color" dialog
    long onCmdOpenColorDialog(FXObject*, FXSelector, void*);

    /// @brief called when user press "open allow" dialog
    long onCmdOpenAllowDialog(FXObject*, FXSelector, void*);

    /// @brief called when user press reparent button
    long onCmdReparent(FXObject*, FXSelector, void*);

    /// @brief called when user press "inspect parent"
    long onCmdInspectParent(FXObject*, FXSelector, void*);

    /// @brief called when user press "move lane up"
    long onCmdMoveLaneUp(FXObject*, FXSelector, void*);

    /// @brief called when user press "move lane down"
    long onCmdMoveLaneDown(FXObject*, FXSelector, void*);

    /// @}

protected:
    /// @brief default constructor (needed for FOX)
    GNEAttributesEditorRow();

    /// @brief get value from edited ACs
    const std::string getAttributeValue(const bool enabled) const;

    /// @brief show attribute toogle enable
    void showAttributeToggleEnable(const GNEAttributeProperties* attrProperty, const bool value);

    /// @brief show attribute reparent
    void showAttributeReparent(const bool enabled);

    /// @brief show attribute parent
    void showAttributeInspectParent(const GNEAttributeProperties* attrProperty, const bool enabled);

    /// @brief show attribute vClass
    void showAttributeVClass(const GNEAttributeProperties* attrProperty, const bool enabled);

    /// @brief show attribute color
    void showAttributeColor(const GNEAttributeProperties* attrProperty, const bool enabled);

    /// @brief show attribute label
    void showAttributeLabel(const GNEAttributeProperties* attrProperty);

    /// @brief show value for check button
    void showValueCheckButton(const std::string& value, const bool enabled, const bool computed);

    /// @brief show value for combo Box
    void showValueComboBox(const GNEAttributeProperties* attrProperty, const std::string& value, const bool enabled, const bool computed);

    /// @brief show value for strings
    void showValueString(const std::string& value, const bool enabled, const bool computed);

    /// @brief show move lane buttons
    void showMoveLaneButtons(const std::string& laneID);

    /// @brief check if enable or disable all elements depending of current supermode or forceDisable
    void enableElements(const GNEAttributeProperties* attrProperty, const bool forceDisable);

    /// @brief check if the given attribute is enabled
    bool isAttributeEnabled(const GNEAttributeProperties* attrProperty) const;

    /// @brief check junction merging
    /// bool mergeJunction(SumoXMLAttr attr, const std::string& newVal) const;

private:
    /// @brief pointer to attribute table parent
    GNEAttributesEditor* myAttributeTable;

    /// @brief edited attribute
    SumoXMLAttr myAttribute = SUMO_ATTR_NOTHING;

    /// @brief pointer to attribute label
    MFXLabelTooltip* myAttributeLabel = nullptr;

    /// @brief pointer to attribute menu check
    FXCheckButton* myAttributeToggleEnableCheckButton = nullptr;

    /// @brief pointer to attribute reparent button
    MFXButtonTooltip* myAttributeReparentButton = nullptr;

    /// @brief pointer to attribute inspect parent button
    MFXButtonTooltip* myAttributeInspectParentButton = nullptr;

    /// @brief pointer to attribute vClass button
    MFXButtonTooltip* myAttributeVClassButton = nullptr;

    /// @brief pointer to attribute color button
    MFXButtonTooltip* myAttributeColorButton = nullptr;

    /// @brief pointer to text field for modify values
    MFXTextFieldTooltip* myValueTextField = nullptr;

    /// @brief pointer to combo box for select choices
    MFXComboBoxIcon* myValueComboBox = nullptr;

    /// @brief pointer to menu check
    FXCheckButton* myValueCheckButton = nullptr;

    /// @brief Button for move lane up
    MFXButtonTooltip* myValueLaneUpButton = nullptr;

    /// @brief Button for move lane down
    MFXButtonTooltip* myValueLaneDownButton = nullptr;

    /// @brief Invalidated copy constructor.
    GNEAttributesEditorRow(GNEAttributesEditorRow*) = delete;

    /// @brief Invalidated assignment operator.
    GNEAttributesEditorRow& operator=(GNEAttributesEditorRow*) = delete;
};
