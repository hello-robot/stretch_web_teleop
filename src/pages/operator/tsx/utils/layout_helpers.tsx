import {
    ParentComponentDefinition,
    ComponentDefinition,
    ComponentType,
    LayoutDefinition,
    LayoutGridDefinition,
} from "operator/tsx/utils/component_definitions";

/**
 * Moves a component from an old path to a new path
 * @param oldPath path to where the object was/is
 * @param newPath path to where the component should be moved
 * @param layout the entire layout structure
 * @returns the new path to the moved object (note: it's possible this is different
 *      than the `newPath`)
 */
export function moveInLayout(
    oldPath: string,
    newPath: string,
    layout: LayoutDefinition,
): string {
    // Get the child and its old parent
    console.log("old path", oldPath);
    console.log("newpath", newPath);
    const oldPathSplit = oldPath.split("-");
    const oldParent = getParent(oldPathSplit, layout);
    console.log("old parent", oldParent);
    let oldChildIdx = +oldPathSplit.slice(-1);
    console.log("old child index", oldChildIdx);
    const temp = getChildFromParent(oldParent, oldChildIdx);
    console.log("temp", temp);

    // Get the new parent
    let newPathSplit = newPath.split("-");
    const newChildIdx = +newPathSplit.slice(-1);
    console.log("newChildIdx", newChildIdx);
    const newParent = getParent(newPathSplit, layout);
    console.log("newparent", newParent);

    // Remove the child from the old parent
    removeChildFromParent(oldParent, oldPathSplit, oldChildIdx, layout);

    // Put the child into the new parent
    putChildInParent(newParent, temp, newChildIdx);
    console.log("after adding child", newParent.children);

    // Same parent and moved to lower index, previous position index is now higher
    if (oldParent === newParent && oldChildIdx > newChildIdx) oldChildIdx++;

    // If newPath only contains one element it is a layout grid with one panel
    // Add 0 to the path to point to select first child in layout grid
    // if (newParent.type == ComponentType.LayoutGrid) {
    //     console.log(newPathSplit)
    if (newParent.type == ComponentType.LayoutGrid) {
        if (+newPathSplit[0] === layout.children.length)
            newPathSplit[0] = (+newPathSplit[0] - 1).toString();
        return newPathSplit.join("-");
    }
    if (newParent.type == ComponentType.Layout) {
        if (+newPathSplit[0] === layout.children.length)
            newPathSplit[0] = (+newPathSplit[0] - 1).toString();
        if (newPathSplit.length === 1) newPathSplit.push("0");
        return newPathSplit.join("-");
    }
    // }

    // Check if removing the old path changes the new path
    // note: this happens when the old path was a sibling with a lower index to
    //       any node in the
    if (newPathSplit.length < oldPathSplit.length) return newPath;

    const oldPathLastIdx = oldPathSplit.length - 1;
    const oldPrefix = oldPathSplit.slice(0, oldPathLastIdx);
    const newPrefix = newPathSplit.slice(0, oldPathLastIdx);
    const sameParent = oldPrefix.every(
        (val, index) => val === newPrefix[index],
    );

    if (!sameParent) return newPath;

    // index of new sibling node
    const newCorrespondingIdx = +newPathSplit[oldPathLastIdx];
    if (oldChildIdx < newCorrespondingIdx) {
        // decrease new path index since the old path is deleted
        newPathSplit[oldPathLastIdx] = "" + (+newPathSplit[oldPathLastIdx] - 1);
        console.log("updated path", newPathSplit.join("-"));
    }

    return newPathSplit.join("-");
}

/**
 * Adds a new component to the layout
 * @param definition definition of the component to add into the layout
 * @param newPath path where to add the new component
 * @param layout the entire layout structure
 */
export function addToLayout(
    definition: ComponentDefinition,
    newPath: string,
    layout: LayoutDefinition,
) {
    let newPathSplit = newPath.split("-");
    const newChildIdx = +newPathSplit.slice(-1);
    const newParent = getParent(newPathSplit, layout);
    putChildInParent(newParent, definition, newChildIdx);

    // If newPath only contains one element it is a layout grid with one panel
    // Add 0 to the path to point to select first child in layout grid
    return newPathSplit.length == 1 ? newPath + "-0" : newPath;
}

/**
 * Deletes a component from the layout
 * @param path path to the component to delete
 * @param layout the entire layout structure
 */
export function removeFromLayout(path: string, layout: LayoutDefinition) {
    const splitPath = path.split("-");
    const childIdx = +splitPath.slice(-1);
    const parent = getParent(splitPath, layout);
    removeChildFromParent(parent, splitPath, childIdx, layout);
}

/**
 * Removes the child from the
 * @param parent parent component definition
 * @param childSplitPath path to child element, split into list of indices
 * @param childIdx index of child (last element in `splitPath`)
 * @param layout entire layout structure
 */
function removeChildFromParent(
    parent: ParentComponentDefinition,
    childSplitPath: string[],
    childIdx: number,
    layout: LayoutDefinition,
) {
    // Remove the child from the parent
    parent.children.splice(childIdx, 1);
    // If it was the last child, also remove the parent. Continue iteratively.
    let i = -2;
    let parentIdx: number;
    while (i >= -childSplitPath.length && parent.children.length === 0) {
        // A tab is the only parent type that can have no children
        if (parent.type === ComponentType.SingleTab) {
            break;
        }
        parentIdx = +childSplitPath.slice(i, i + 1);
        const grandparent = getParent(childSplitPath.slice(0, i + 1), layout);
        grandparent.children.splice(parentIdx, 1);
        parent = grandparent;
        i -= 1;
    }
}

/**
 * Gets the parent definition for the given child path
 * @param splitPath path to child element, split into a list of indices
 * @param layout the layout object
 * @returns the parent definition
 */
function getParent(
    splitPath: string[],
    layout: ParentComponentDefinition,
): ParentComponentDefinition {
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
function getChildFromParent(
    parent: ParentComponentDefinition,
    childIdx: number,
): ComponentDefinition {
    return parent.children[childIdx];
}

/**
 * Inserts a child into the parent at a certain index
 * @param parent definition of parent component
 * @param child definition of child component
 * @param childIdx index where to insert the child
 */
function putChildInParent(
    parent: ParentComponentDefinition,
    child: ComponentDefinition,
    childIdx: number,
) {
    if (parent.children) {
        parent.type == ComponentType.Layout
            ? parent.children.splice(childIdx, 0, {
                  type: ComponentType.LayoutGrid,
                  children: [child],
              } as LayoutGridDefinition)
            : parent.children.splice(childIdx, 0, child);
    } else {
        parent.children = [child];
    }
}
