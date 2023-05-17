import { ParentComponentDefinition, ComponentDefinition } from "operator/tsx/utils/componentdefinitions";

/**
 * Gets the parent definition for the given child path
 * @param splitPath path to child element, split into a list of indices
 * @param layout the layout object
 * @returns the parent definition
 */
const getParent = (splitPath: string[], layout: ParentComponentDefinition): ParentComponentDefinition => {
    let pathIdx = 0;
    let parent: ParentComponentDefinition = layout;
    while (pathIdx < splitPath.length - 1) {
        const childIdx = +splitPath[pathIdx];
        parent = parent.children[childIdx] as ParentComponentDefinition;
        pathIdx++;
    }
    return parent!;
}

/**
 * Gets one of the children from a parent
 * @param parent definition of the parent component
 * @param childIdx index of the child element to retrieve
 * @returns the child component definition
 */
const getChildFromParent = (parent: ParentComponentDefinition, childIdx: number): ComponentDefinition => {
    return parent.children[childIdx];
}

/**
 * Inserts a child into the parent at a certain index
 * @param parent definition of parent component
 * @param child definition of child component
 * @param childIdx index where to insert the child
 */
const putChildInParent = (parent: ParentComponentDefinition, child: ComponentDefinition, childIdx: number) => {
    parent.children.splice(childIdx, 0, child);
}

/**
 * Removes a child from the parent definition at a certain index
 * @param parent definition of parent component
 * @param childIdx index of the child to remove
 */
const removeChildFromParent = (parent: ParentComponentDefinition, childIdx: number) => {
    parent.children.splice(childIdx, 1);
}

/**
 * Moves a component from an old path to a new path
 * @param oldPath path to where the object was/is
 * @param newPath path to where the component should be moved
 * @param layout the entire layout structure
 * @returns the new path to the moved object (note: it's possible this is different
 *      than the `newPath`)
 */
export const moveInLayout = (oldPath: string, newPath: string, layout: ParentComponentDefinition): string => {
    // Get the child and its old parent
    console.log('old path', oldPath);
    console.log('newpath', newPath);
    const oldPathSplit = oldPath.split('-');
    const oldParent = getParent(oldPathSplit, layout);
    console.log('old parent', oldParent);
    let oldChildIdx = +oldPathSplit.slice(-1);
    console.log('old child index', oldChildIdx);
    const temp = getChildFromParent(oldParent, oldChildIdx);
    console.log('temp', temp)

    // Get the new parent
    let newPathSplit = newPath.split('-');
    const newChildIdx = +newPathSplit.slice(-1);
    console.log('newChildIdx', newChildIdx)
    const newParent = getParent(newPathSplit, layout);
    console.log('newparent', newParent)

    // Put the child into the new parent
    putChildInParent(newParent, temp, newChildIdx);
    console.log('after adding child', newParent.children);

    // Same parent and moved to lower index, previous position index is now higher
    if (oldParent === newParent && oldChildIdx > newChildIdx)
        oldChildIdx++;

    // Remove the child from the old parent
    removeChildFromParent(oldParent, oldChildIdx);

    // Check if removing the old path changes the new path
    // note: this happens when the old path was a sibling with a lower index to
    //       any node in the 
    if (newPathSplit.length < oldPathSplit.length)
        return newPath;

    const oldPathLastIdx = oldPathSplit.length - 1;
    const oldPrefix = oldPathSplit.slice(0, oldPathLastIdx);
    const newPrefix = newPathSplit.slice(0, oldPathLastIdx)
    const sameParent = oldPrefix.every((val, index) => val === newPrefix[index])

    if (!sameParent)
        return newPath;

    // index of new sibling node
    const newCorrespondingIdx = +newPathSplit[oldPathLastIdx];
    if (oldChildIdx < newCorrespondingIdx) {
        // decrease new path index since the old path is deleted
        newPathSplit[oldPathLastIdx] = "" + (+newPathSplit[oldPathLastIdx] - 1);
        console.log('updated path', newPathSplit.join('-'))
    }
    return newPathSplit.join('-');
}

/**
 * Adds a new component to the layout
 * @param activeDef definition of the component to add into the layout
 * @param newPath path where to add the new component
 * @param layout the entire layout structure
 */
export const addToLayout = (
    activeDef: ComponentDefinition,
    newPath: string,
    layout: ParentComponentDefinition
) => {
    let newPathSplit = newPath.split('-');
    const newChildIdx = +newPathSplit.slice(-1);
    const newParent = getParent(newPathSplit, layout);
    putChildInParent(newParent, activeDef, newChildIdx);
}

/**
 * Deletes a component from the layout
 * @param path path to the component to delete
 * @param layout the entire layout structure
 */
export const removeFromLayout = (
    path: string,
    layout: ParentComponentDefinition
) => {
    const splitPath = path.split('-');
    const childIdx = +splitPath.slice(-1);
    const parent = getParent(splitPath, layout);
    removeChildFromParent(parent, childIdx)
}