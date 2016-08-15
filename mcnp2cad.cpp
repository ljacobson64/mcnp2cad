#include <iostream>
#include <fstream>
#include <string>
#include <stdexcept>
#include <cctype>
#include <vector>
#include <set>
#include <map>
#include <sstream>
#include <algorithm>

#include <cassert>

#include "iGeom.h"
#include "geometry.hpp"
#include "MCNPInput.hpp"
#include "options.hpp"
#include "volumes.hpp"
#include "ProgOptions.hpp"
#include "GeometryContext.hpp"
#include "version.hpp"

void debugSurfaceDistances( InputDeck& deck, std::ostream& out = std::cout ){

  InputDeck::surface_card_list& surfaces = deck.getSurfaces();
  for( InputDeck::surface_card_list::iterator i = surfaces.begin(); i!=surfaces.end(); ++i){
    try{
      const SurfaceVolume& s = makeSurface(*i);
      out << "S" << (*i)->getIdent() << " distance from origin: " << s.getFarthestExtentFromOrigin() << std::endl;
    }
    catch(std::runtime_error& e){
      std::cerr << "Error debugging surface distances: " << e.what() << std::endl;
      throw;
    }
  }
  
}

std::string mcnp2cad_version(bool full = true);

extern GeometryContext context;

struct program_option_struct Gopt;

int main(int argc, char* argv[]){

  // set default options
  Gopt.verbose = Gopt.debug = false;
  Gopt.infinite_lattice_extra_effort = false;
  Gopt.tag_materials = true;
  Gopt.tag_importances = true;
  Gopt.tag_cell_IDs = true;
  Gopt.make_graveyard = true;
  Gopt.imprint_geom = true;
  Gopt.merge_geom = true;
  Gopt.input_file = "";
  Gopt.output_file = OPT_DEFAULT_OUTPUT_FILENAME;
  Gopt.igeom_init_options = "";
  Gopt.override_tolerance = false;
  Gopt.uwuw_names = false;

  bool DiFlag = false, DoFlag = false;

  ProgOptions po("mcnp2cad " + mcnp2cad_version(false) +  ": An MCNP geometry to CAD file converter");
  po.setVersion( mcnp2cad_version() );

  po.addOpt<void>("extra-effort,e","Use extra effort to get infinite lattices right (may be slow)", 
                  &Gopt.infinite_lattice_extra_effort );
  po.addOpt<void>("verbose,v", "Verbose output", &Gopt.verbose );
  po.addOpt<void>("debug,D", "Debugging (very verbose) output", &Gopt.debug );
  po.addOpt<void>("Di", "Debug output for MCNP parsing phase only", &DiFlag);
  po.addOpt<void>("Do","Debug output for iGeom output phase only", &DoFlag);

  po.addOptionHelpHeading( "Options controlling CAD output:" );
  po.addOpt<std::string>(",o", "Give name of output file. Default: " + Gopt.output_file, &Gopt.output_file );
  po.addOpt<double>("tol,t", "Specify a tolerance for merging surfaces", &Gopt.specific_tolerance );
  po.addOpt<void>("skip-mats,M", "Do not tag materials using group names", 
                  &Gopt.tag_materials, po.store_false );
  po.addOpt<void>("skip-imps,P", "Do not tag cell importances using group names",
                  &Gopt.tag_importances, po.store_false );
  po.addOpt<void>("skip-nums,N", "Do not tag cell numbers using body names",
                  &Gopt.tag_cell_IDs, po.store_false );
  po.addOpt<void>("skip-merge,E", "Do not merge the geometry",
                  &Gopt.merge_geom, po.store_false );
  po.addOpt<void>("skip-imprint,I", "Do not imprint the geometry; implies skip-merge",
                  &Gopt.imprint_geom, po.store_false );
  po.addOpt<void>("skip-graveyard,G", "Do not bound the geometry with a `graveyard' bounding box",
                  &Gopt.make_graveyard, po.store_false );
  po.addOpt<void>("uwuw-names,U", "Use a UWUW compatible name scheme for material groups,"
                                   "i.e. 'mat:mX/rho:Y' where X is material number is Y is density",
                  &Gopt.uwuw_names, po.store_true );

#ifdef USING_CGMA
  po.addOptionHelpHeading ("Options controlling CGM library:");
  po.addOpt<int>("geomver","Override geometry export engine version");
  po.addOptionHelpHeading ("    (use --geomver 1600 for backward compatibility w/ Cubit 10.2)");
  po.addOpt<void>("Cv", "Verbose messages from CGM" );
  po.addOpt<void>("Cq", "Silence warning messages from CGM" );
  po.addOpt<void>("CIq","Silence ERROR messages from CGM when doing intersect tests.");
  po.addOptionHelpHeading( "         (May be useful for infinite lattices, but use cautiously)" );
#endif

  po.addRequiredArg( "input_file", "Path to MCNP geometry input file", &Gopt.input_file );

  po.parseCommandLine( argc, argv );

  if( po.numOptSet( "tol,t" ) ){
    Gopt.override_tolerance = true;
    if( Gopt.specific_tolerance <= 0.0 || Gopt.specific_tolerance > .1 ){
      std::cerr << "Warning: you seem to have specified an unusual tolerance (" 
                << Gopt.specific_tolerance << ")." << std::endl;
    }
  }

#ifdef USING_CGMA

  // Enable the info_flag only if --Cv is requested
  if(po.numOptSet( "Cv" )){
    CubitMessage::instance()->set_info_flag( true );
  }
  else{
    CubitMessage::instance()->set_info_flag( false );
  }

  // Silence warnings if --Cq is set
  if(po.numOptSet( "Cq" )){    
    CubitMessage::instance()->set_warning_flag( false );
  }

  // Enable silent intersection errors if --CIq is set
  if(po.numOptSet( "CIq" )){
    CGMA_opt_inhibit_intersect_errs = true;
  }

#endif

  if( Gopt.merge_geom && !Gopt.imprint_geom ) {
    std::cerr << "Warning: cannot merge geometry without imprinting, will skip merge too." << std::endl;
  }

  std::ifstream input(Gopt.input_file.c_str(), std::ios::in );
  if( !input.is_open() ){
    std::cerr << "Error: couldn't open file \"" << Gopt.input_file << "\"" << std::endl;
    return 1;
  }
  
  std::cout << "Reading input file..." << std::endl;

  // if --Di and not -D, set debugging to be true for InputDeck::build() call only
  if( DiFlag && !OPT_DEBUG){
    Gopt.debug = true;
  }
  else{ DiFlag = false; }

  InputDeck& deck = InputDeck::build(input);
  std::cout << "Done reading input." << std::endl;

  // turn off debug if it was set by --Di only
  if( DiFlag ){ Gopt.debug = false; }
  
  if( DoFlag && !OPT_DEBUG ){ Gopt.debug = true; }

  if( OPT_DEBUG ){ 
    debugSurfaceDistances( deck );
  }

  iGeom_Instance igm;
  int igm_result; 

  iGeom_newGeom( Gopt.igeom_init_options.c_str(), &igm, &igm_result, Gopt.igeom_init_options.length() );
  CHECK_IGEOM( igm_result, "Initializing iGeom");

#ifdef USING_CGMA
  int export_vers;
  if( po.getOpt( "geomver", &export_vers) ){
    if( CUBIT_SUCCESS == GeometryQueryTool::instance()->set_export_allint_version( export_vers ) ){
      std::cout << "Set export engine version to " << export_vers << std::endl; 
    }
    // on failure, an error message will be printed by CGM
  }

#endif

  GeometryContext* context = GeometryContext::getInstance(igm, deck);
  context->createGeometry();

  return 0;
    
}

/** Return the version string
 *
 * Return only numbers ("1.2.3") if full is false, else dated format
 */
std::string mcnp2cad_version( bool full ){
  std::stringstream str;
  str << (full ? "mcnp2cad version " : "")
      << MCNP2CAD_VERSION_MAJOR << "." 
      << MCNP2CAD_VERSION_MINOR << "." 
      << MCNP2CAD_VERSION_REV;
  if(full)
      str << "\nCompiled on " << __DATE__ << " at " << __TIME__ ;
  return str.str();
}
