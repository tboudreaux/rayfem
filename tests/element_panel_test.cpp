#include "../src/app/element_panel.hpp"
#include <iostream>
#include <stdexcept>

namespace {
void Check(bool condition,const char* message) {if(!condition)throw std::runtime_error(message);}
std::shared_ptr<RayFEM::RenderFrame> Frame(const std::shared_ptr<mfem::Mesh>& mesh,unsigned index=0) {
    auto frame=std::make_shared<RayFEM::RenderFrame>();frame->mesh=mesh;
    frame->session_id=11;frame->mesh_id=12;frame->mesh_revision=13;frame->metadata.frame_index=index;
    return frame;
}
}

int main() {
    try {
        auto mesh=std::make_shared<mfem::Mesh>(mfem::Mesh::MakeCartesian3D(3,1,1,mfem::Element::HEXAHEDRON));
        auto first=Frame(mesh),second=Frame(mesh,1);
        ElementPanel panel;panel.Context(first);const auto initial=panel.Revision();
        // Mouse-event modifiers must work without a separate keyboard event
        // or linking the macOS event monitor into these headless UI tests.
        RayFEM::UI::detail::mouseSelectionShiftGetter=[] {return false;};
        panel.Select(1,RayFEM::UI::MouseSelectionShift());
        RayFEM::UI::detail::mouseSelectionShiftGetter=[] {return true;};
        panel.Select(2,RayFEM::UI::MouseSelectionShift());
        Check(panel.Selected()==std::vector<int>({1,2}),"Mouse-only Shift adds to selection");
        panel.Select(1,RayFEM::UI::MouseSelectionShift());
        Check(panel.Selected()==std::vector<int>{2},"Mouse-only Shift toggles selected cells");
        RayFEM::UI::detail::mouseSelectionShiftGetter=[] {return false;};
        panel.Select(0,RayFEM::UI::MouseSelectionShift());
        Check(panel.Selected()==std::vector<int>{0},"Following unmodified click clears prior Shift state");
        RayFEM::UI::detail::mouseSelectionShiftGetter=nullptr;
        panel.Select(std::nullopt,false);
        panel.Select(2,false);panel.Select(0,true);
        Check(panel.Selected()==std::vector<int>({0,2}),"Shift selection is sorted");
        panel.SetVisible(2,false);panel.SetVisible(0,false);
        Check(panel.Hidden()==std::vector<int>({0,2}),"Hidden IDs are sorted");
        Check(panel.Selected()==std::vector<int>({0,2}),"Hiding preserves selected IDs");
        const auto current=panel.Revision();
        Check(current>initial,"Changes invalidate overlays");
        panel.Context(second);
        Check(panel.Revision()==current && panel.Selected().size()==2 && panel.Hidden().size()==2,
              "Streamed frames of the same mesh preserve state");
        panel.Select(std::nullopt,true);
        Check(panel.Revision()==current,"Shift-clicking empty space preserves selection");
        panel.Select(0,true);
        Check(panel.Selected()==std::vector<int>{2},"Shift-click toggles membership");
        const auto unchanged=panel.Revision();panel.Select(2,false);panel.SetVisible(2,false);panel.Select(99,false);
        Check(panel.Revision()==unchanged,"Unchanged and invalid operations do not invalidate overlays");
        panel.Select(std::nullopt,false);
        Check(panel.Selected().empty() && panel.Hidden().size()==2,"Clearing selection preserves visibility");
        panel.SetVisible(0,true);
        Check(panel.Hidden()==std::vector<int>{2},"Individual restore");
        panel.Select(1,false);const auto beforeRevision=panel.Revision();
        auto changed=Frame(mesh,2);changed->mesh_revision++;
        panel.Context(changed);
        Check(panel.Selected().empty() && panel.Hidden().empty() && panel.Revision()>beforeRevision,
              "New mesh revision resets IDs and advances overlay revision monotonically");
        panel.Select(1,false);panel.SetVisible(1,false);
        auto otherSession=Frame(mesh,3);otherSession->session_id++;
        panel.Context(otherSession);
        Check(panel.Selected().empty() && panel.Hidden().empty(),"Other session cannot inherit selected cells");
        panel.Select(1,false);panel.SetVisible(1,false);
        auto otherMesh=Frame(std::make_shared<mfem::Mesh>(*mesh),4);otherMesh->session_id=otherSession->session_id;
        panel.Context(otherMesh);
        Check(panel.Selected().empty() && panel.Hidden().empty(),"Changed physical mesh resets state even if sender reused IDs");
        panel.Select(1,false);const auto beforeClear=panel.Revision();panel.Context(nullptr);
        Check(panel.Selected().empty() && panel.Hidden().empty() && panel.Revision()>beforeClear,"Unloading mesh clears state");
        panel.Context(first);panel.Select(1,false);panel.SetVisible(1,false);
        mesh->UniformRefinement();panel.Context(first);
        Check(panel.Selected().empty() && panel.Hidden().empty(),"In-place MFEM topology changes invalidate element IDs");
        std::weak_ptr<RayFEM::RenderFrame> weak=first;first.reset();
        Check(weak.expired(),"Panel does not retain full frames");
        std::cout<<"PASS element-panel selection, visibility, frame persistence, identity isolation and overlay invalidation\n";
        return 0;
    }catch(const std::exception& error){std::cerr<<"Element panel: "<<error.what()<<'\n';return 1;}
}
