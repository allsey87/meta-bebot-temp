
#include "frame_annotator.h"

#include "tag.h"
#include "block.h"
#include "target.h"
#include "structure.h"

/* for cv::projectPoints */
#include <opencv2/calib3d/calib3d.hpp>


/****************************************/
/****************************************/

void CFrameAnnotator::Annotate(const STag& s_tag,
                               const cv::Scalar& c_color,
                               int n_thickness,
                               const std::string& s_text) {
   const std::vector<std::pair<double, double>>& Corners = s_tag.Corners;
   for(uint8_t un_corner_idx = 0; un_corner_idx < 4; un_corner_idx++) {
      m_vecLines.emplace_back([=] (cv::Mat& c_frame) {
         cv::line(c_frame,
                  cv::Point2d(Corners[un_corner_idx].first, Corners[un_corner_idx].second),
                  cv::Point2d(Corners[(un_corner_idx + 1) % 4].first, Corners[(un_corner_idx + 1) % 4].second),
                  c_color,
                  n_thickness);
      });
   }
   if(!s_text.empty()) {
      Label(cv::Point2d(s_tag.Center.first + 10, s_tag.Center.second + 10), s_text);
   }
}

/****************************************/
/****************************************/

void CFrameAnnotator::Annotate(const SBlock& s_block,
                               const cv::Scalar& c_color,
                               const std::string& s_text) {
   /* project block points */
   std::vector<cv::Point2d> vecOutputImagePoints;
   cv::projectPoints(m_vecBlockVertices,
                     s_block.RotationVector,
                     s_block.TranslationVector,
                     m_cCameraMatrix,
                     m_cDistortionParameters,
                     vecOutputImagePoints);
   /* project block points */
   m_vecLines.emplace_back([=] (cv::Mat& c_frame) {
      cv::line(c_frame, vecOutputImagePoints[0], vecOutputImagePoints[1], c_color, 1);
      cv::line(c_frame, vecOutputImagePoints[1], vecOutputImagePoints[2], c_color, 1);
      cv::line(c_frame, vecOutputImagePoints[2], vecOutputImagePoints[3], c_color, 1);
      cv::line(c_frame, vecOutputImagePoints[3], vecOutputImagePoints[0], c_color, 1);

      cv::line(c_frame, vecOutputImagePoints[4], vecOutputImagePoints[5], c_color, 1);
      cv::line(c_frame, vecOutputImagePoints[5], vecOutputImagePoints[6], c_color, 1);
      cv::line(c_frame, vecOutputImagePoints[6], vecOutputImagePoints[7], c_color, 1);
      cv::line(c_frame, vecOutputImagePoints[7], vecOutputImagePoints[4], c_color, 1);

      cv::line(c_frame, vecOutputImagePoints[0], vecOutputImagePoints[4], c_color, 1);
      cv::line(c_frame, vecOutputImagePoints[1], vecOutputImagePoints[5], c_color, 1);
      cv::line(c_frame, vecOutputImagePoints[2], vecOutputImagePoints[6], c_color, 1);
      cv::line(c_frame, vecOutputImagePoints[3], vecOutputImagePoints[7], c_color, 1);
   });

   for(const STag& s_tag : s_block.Tags) {
      Annotate(s_tag, c_color, 2);
   }
   for(const STag& s_tag : s_block.HackTags) {
      Annotate(s_tag, c_color, 1);
   }

   if(!s_text.empty()) {
      Label(cv::Point2d(s_block.Coordinates.GetX() + 10, s_block.Coordinates.GetY() + 10), s_text);
   }
}

/****************************************/
/****************************************/

void CFrameAnnotator::Annotate(const STarget& s_target,
                               const cv::Scalar& c_color,
                               const std::string& s_text) {
   /* draw the block */
   Annotate(s_target.Observations.front(), cv::Scalar(c_color * 0.75));
   /* draw a trace showing previous observations */
   for(SBlock::TConstListIterator it_block = std::begin(s_target.Observations);
       it_block != std::end(s_target.Observations);
       it_block++) {

      const argos::CVector2& cCoordinates = it_block->Coordinates;
      m_vecLines.emplace_back([=] (cv::Mat& c_frame) {
         cv::circle(c_frame, 
                    cv::Point2d(cCoordinates.GetX(), cCoordinates.GetY()),
                    2.5f,
                    c_color);
      });
      SBlock::TConstListIterator itNextBlock = std::next(it_block);      
      if(itNextBlock != std::end(s_target.Observations)) {
         const argos::CVector2& cFrom = it_block->Coordinates;
         const argos::CVector2& cTo = itNextBlock->Coordinates;
         m_vecLines.emplace_back([=] (cv::Mat& c_frame) {
            cv::line(c_frame,
                     cv::Point2d(cFrom.GetX(), cFrom.GetY()),
                     cv::Point2d(cTo.GetX(), cTo.GetY()),
                     cv::Scalar(c_color),
                     1);
         });
      }
   }
   if(!s_text.empty()) {
      const argos::CVector2& cCoords = std::begin(s_target.Observations)->Coordinates;
      Label(cv::Point2d(cCoords.GetX() + 10, cCoords.GetY() + 10), s_text);
   }  
}

/****************************************/
/****************************************/

void CFrameAnnotator::Annotate(const SStructure& s_structure,
                               const cv::Scalar& c_color,
                               const std::string& s_text) {

   /* draw the block */
   for(const STarget::TConstListIterator& s_target_iter : s_structure.Members) {
      //Annotate(s_target_iter->Observations.front(), c_color);
      Annotate(*s_target_iter, c_color, "[" + std::to_string(s_target_iter->Id) + "]");
   }
   /*
   if(!s_text.empty()) {
      const argos::CVector2& cCoords = std::begin(s_target.Observations)->Coordinates;
      Label(cv::Point2d(cCoords.GetX() + 10, cCoords.GetY() + 10), s_text);
   } 
   */ 
}

/****************************************/
/****************************************/

void CFrameAnnotator::Label(const cv::Point2d& c_origin,
                            const std::string& str_text) {

   auto tFont = cv::FONT_HERSHEY_SIMPLEX;
   double fScale = 0.5;
   int nThickness = 1;
   int nBaseline = 0;
   int nBorderThickness = 5;

   cv::Size cTextSize = cv::getTextSize(str_text, tFont, fScale, nThickness, &nBaseline);

   m_vecLabelBackgrounds.emplace_back([=] (cv::Mat& c_frame) {
      cv::rectangle(c_frame,
                    c_origin + cv::Point2d(0, nBaseline) + cv::Point2d(-nBorderThickness, nBorderThickness),
                    c_origin + cv::Point2d(cTextSize.width, -cTextSize.height) + cv::Point2d(nBorderThickness, -nBorderThickness),
                    cv::Scalar(0,0,0),
                    -1);
   });

   m_vecLabels.emplace_back([=] (cv::Mat& c_frame) {
      cv::putText(c_frame, str_text, c_origin, tFont, fScale, cv::Scalar(255,255,255), nThickness, 8);
   });
}

/****************************************/
/****************************************/

void CFrameAnnotator::WriteToFrame(cv::Mat& c_frame) {
   for(auto& fn_draw : m_vecLines) {
      fn_draw(c_frame);
   }
   for(auto& fn_draw : m_vecLabelBackgrounds) {
      fn_draw(c_frame);
   }
   for(auto& fn_draw : m_vecLabels) {
      fn_draw(c_frame);
   }
}

/****************************************/
/****************************************/

void CFrameAnnotator::Clear() {
   m_vecLabels.clear();
   m_vecLines.clear();
   m_vecLabelBackgrounds.clear();
}

/****************************************/
/****************************************/



