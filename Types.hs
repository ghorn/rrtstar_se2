{-# OPTIONS_GHC -Wall -Wno-unused-top-binds -Werror #-}

module Types
  ( Se2Tree
  , insertPointInTree
  ) where

data XCoord = XCoord Double
data YCoord = YCoord Double
data QCoord = QCoord Double

data Point = Point XCoord YCoord QCoord
data PointBounds = PointBounds Point Point -- (XCoord, XCoord) (YCoord, YCoord) (QCoord, QCoord)

--data XTree = XTree {xsplitLine :: XCoord, xleftTree :: Either Point YTree, xrightTree :: Either Point YTree}
--data YTree = YTree {ysplitLine :: YCoord, yleftTree :: Either Point ZTree, yrightTree :: Either Point ZTree}
--data QTree = QTree {qsplitLine :: QCoord, qleftTree :: Either Point QTree, qrightTree :: Either Point QTree}

data Axis = X | Y | Q

data Node = NodeEmpty | NodePoint Point | NodeTree Se2Tree



data Se2Tree
  = Se2Tree
    { _se2TreeSplitAxis :: Axis
    , _se2TreeSplitAxisValue :: Double
    , _se2TreeAxisBounds :: PointBounds -- (Double, Double)
    , _se2TreeLeftTree  :: Node -- Maybe (Either Point Se2Tree)
    , _se2TreeRightTree :: Node -- Maybe (Either Point Se2Tree)
    }

coordLessThanOrEq :: Axis -> Point -> Point -> Bool
coordLessThanOrEq X (Point (XCoord x0) _ _) (Point (XCoord x1) _ _) = x0 <= x1
coordLessThanOrEq Y (Point _ (YCoord y0) _) (Point _ (YCoord y1) _) = y0 <= y1
coordLessThanOrEq Q (Point _ _ (QCoord q0)) (Point _ _ (QCoord q1)) = q0 <= q1

insertPointInTree :: Point -> Se2Tree -> Se2Tree
insertPointInTree = undefined
--insertPointInTree point (Se2Tree axis splitValue (PointBounds lb ub) left right)
--  | coordLessThanOrEq axis point splitValue =
--      Se2Tree axis splitValue (PointBounds lb ub) (insertPointInNode point (PointBounds lb splitValue) left) right
--  | otherwise =
--      Se2Tree axis splitValue (PointBounds lb ub) left (insertPointInNode point (PointBounds splitValue ub) right)

-- TODO(greg): Where should PointBounds live? Answer this after doing the nearest neighbors lookup.
insertPointInNode :: Point -> PointBounds -> Node -> Node
insertPointInNode = undefined
--insertPointInNode axis newPoint pointBounds NodeEmpty = NodePoint newPoint pointBounds
--insertPointInNode axis newPoint _pointBounds (NodeTree tree) = NodeTree $ insertPointInTree newPoint tree
--insertPointInNode axis newPoint pointBounds (NodePoint existingPoint) = NodeTree
--  Se2Tree
--  { se2TreeSplitAxis = axis
--    se2TreeSplitValue = 
--  } -- $ insertPointInTree newPoint tree

newtype DistanceSquared = DistanceSquared Double

nearestPointTo :: Maybe (Point, DistanceSquared) -> Point -> Se2Tree -> Maybe (Point, DistanceSquared)
nearestPointTo bestPointSoFar point (Se2Tree axis splitValue bounds NodeEmpty NodeEmpty) = bestPointSoFar
nearestPointTo bestPointSoFar point (Se2Tree axis splitValue bounds left right) = bestPointSoFar
--nearestPointTo bestPointSoFar point (Se2Tree axis splitValue bounds NodeEmpty NodeEmpty) = bestPointSoFar
-- getPointsWithinDistanceOf :: Point -> 
