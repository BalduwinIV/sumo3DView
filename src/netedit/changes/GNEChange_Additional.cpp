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
/// @file    GNEChange_Additional.cpp
/// @author  Pablo Alvarez Lopez
/// @date    Nov 2015
///
// A network change in which a busStop is created or deleted
/****************************************************************************/

#include <netedit/GNENet.h>
#include <netedit/GNEViewNet.h>
#include <netedit/GNEViewParent.h>
#include <netedit/GNEApplicationWindow.h>

#include "GNEChange_Additional.h"

// ===========================================================================
// FOX-declarations
// ===========================================================================

FXIMPLEMENT_ABSTRACT(GNEChange_Additional, GNEChange, nullptr, 0)

// ===========================================================================
// member method definitions
// ===========================================================================

GNEChange_Additional::GNEChange_Additional(GNEAdditional* additional, bool forward) :
    GNEChange(Supermode::NETWORK, additional, forward, additional->isAttributeCarrierSelected()),
    myAdditional(additional) {
    myAdditional->incRef("GNEChange_Additional");
}


GNEChange_Additional::~GNEChange_Additional() {
    // only continue we have undo-redo mode enabled
    if (myAdditional->getNet()->getViewNet()->getViewParent()->getGNEAppWindows()->isUndoRedoAllowed()) {
        myAdditional->decRef("GNEChange_Additional");
        if (myAdditional->unreferenced()) {
            // make sure that additional isn't in net before removing
            if (myAdditional->getNet()->getAttributeCarriers()->retrieveAdditional(myAdditional, false)) {
                // delete additional from net
                myAdditional->getNet()->getAttributeCarriers()->deleteAdditional(myAdditional);
            }
            delete myAdditional;
        }
    }
}


void
GNEChange_Additional::undo() {
    if (myForward) {
        // unselect if mySelectedElement is enabled
        if (mySelectedElement) {
            myAdditional->unselectAttributeCarrier();
        }
        // delete additional from net
        myAdditional->getNet()->getAttributeCarriers()->deleteAdditional(myAdditional);
        // remove element from parent and children
        removeElementFromParentsAndChildren(myAdditional);
    } else {
        // select if mySelectedElement is enabled
        if (mySelectedElement) {
            myAdditional->selectAttributeCarrier();
        }
        // add element in parent and children
        addElementInParentsAndChildren(myAdditional);
        // insert additional into net
        myAdditional->getNet()->getAttributeCarriers()->insertAdditional(myAdditional);
    }
    // require always save additionals
    myAdditional->getNet()->getSavingStatus()->requireSaveAdditionals();
}


void
GNEChange_Additional::redo() {
    if (myForward) {
        // select if mySelectedElement is enabled
        if (mySelectedElement) {
            myAdditional->selectAttributeCarrier();
        }
        // add element in parent and children
        addElementInParentsAndChildren(myAdditional);
        // insert additional into net
        myAdditional->getNet()->getAttributeCarriers()->insertAdditional(myAdditional);
    } else {
        // unselect if mySelectedElement is enabled
        if (mySelectedElement) {
            myAdditional->unselectAttributeCarrier();
        }
        // delete additional from net
        myAdditional->getNet()->getAttributeCarriers()->deleteAdditional(myAdditional);
        // remove element from parent and children
        removeElementFromParentsAndChildren(myAdditional);
    }
    // require always save additionals
    myAdditional->getNet()->getSavingStatus()->requireSaveAdditionals();
}


std::string
GNEChange_Additional::undoName() const {
    if (myForward) {
        return (TL("Undo create ") + myAdditional->getTagStr() + " '" + myAdditional->getID() + "'");
    } else {
        return (TL("Undo delete ") + myAdditional->getTagStr() + " '" + myAdditional->getID() + "'");
    }
}


std::string
GNEChange_Additional::redoName() const {
    if (myForward) {
        return (TL("Redo create ") + myAdditional->getTagStr() + " '" + myAdditional->getID() + "'");
    } else {
        return (TL("Redo delete ") + myAdditional->getTagStr() + " '" + myAdditional->getID() + "'");
    }
}
