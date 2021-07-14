{-# OPTIONS_GHC -Wall -Wno-unused-top-binds -Werror #-}

module OneD
  ( Tree
  , insertPoint
  ) where

type XCoord = Double
--data YCoord = YCoord Double
--data QCoord = QCoord Double
--
data Point = Point XCoord --  YCoord QCoord
type Bounds = (Double, Double) -- Bounds Point --  Point -- (XCoord, XCoord) (YCoord, YCoord) (QCoord, QCoord)

--data XTree = XTree {xsplitLine :: XCoord, xleftTree :: Either Point YTree, xrightTree :: Either Point YTree}
--data YTree = YTree {ysplitLine :: YCoord, yleftTree :: Either Point ZTree, yrightTree :: Either Point ZTree}
--data QTree = QTree {qsplitLine :: QCoord, qleftTree :: Either Point QTree, qrightTree :: Either Point QTree}

data Axis = X --  | Y | Q

data Tree = Empty | Leaf Point | Split Tree Tree

insertPoint :: Point -> Bounds -> Tree -> Tree
insertPoint p _ Empty = Leaf p
insertPoint p@(Point coord) (lb, ub) (Split left right)
  | coord <= mid = Split (insertPoint p (lb, mid) left) right
  | otherwise = Split left (insertPoint p (mid, ub) right)
  where
    mid = 0.5 * (lb + ub)
insertPoint p1@(Point coord0) (lb, ub) (Leaf p0@(Point coord1))
  -- evenly split
  | p0Left && p1Right = Split (Leaf p0) (Leaf p1)
  -- evenly split the other way
  | p1Left && p0Right = Split (Leaf p1) (Leaf p0)
  -- both on left
  | p0Left && p1Left = Split (insertPoint p1 leftBounds (Leaf p0)) Empty
  -- both on right
--  | p0Right && p1Right = 
  | otherwise = Split Empty (insertPoint p1 rightBounds (Leaf p0))
  where
    mid = 0.5 * (lb + ub)
    leftBounds = (lb, mid)
    rightBounds = (mid, ub)
    p0Left = coord0 <= mid
    p0Right = not p0Left
    p1Left = coord1 <= mid
    p1Right = not p1Left

withinRadius :: Double -> Bounds -> [Point] -> Point -> Tree -> [Point]
withinRadius _ _ acc _ Empty = acc
withinRadius radiusSquared _bnds acc p@(Point testPoint) (Leaf (Point candidatePoint))
  | (testPoint - candidatePoint)**2 <= radiusSquared = p:acc
  | otherwise = acc
withinRadius radiusSquared (lb, ub) acc p@(Tree testPoint) (Leaf (Point candidatePoint))
--insertPoint p1@(Point coord0) (lb, ub) (Leaf p0@(Point coord1)) = Split leftTree rightTree
    --leftTree
    --  | p0Left && p1Left = insertPoint p1 leftBounds (Leaf p0)
    --  | p0Left = Leaf p0
    --  | p1Left = Leaf p1
    --  | otherwise = Empty
    --
    --rightTree
    --  | p0Right && p1Right = insertPoint p1 rightBounds (Leaf p0)
    --  | p0Right = Leaf p0
    --  | p1Right = Leaf p1
    --  | otherwise = Empty
