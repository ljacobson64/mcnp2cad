//#ifndef GEOMETRY_CONTEXT_HPP
//#define GEOMETRY_CONTEXT_HPP

#include "iGeom.h"

#include "volumes.hpp"
#include "MCNPInput.hpp"
#include "options.hpp"
#include <sstream>
#include <algorithm>
#include <iostream>
#include <string>
#include <cassert>
#include <set>

#ifdef USING_CGMA
#include <RefEntityFactory.hpp>
#include <Body.hpp>
#include <GeometryQueryTool.hpp>
#include <CubitMessage.hpp>

static bool CGMA_opt_inhibit_intersect_errs = false;

/*  CubitMessage doesn't provide a way to shut up error messages.
 *  For those rare cases when we really want that behavior, we set up 
 *  a message handler that just drops messages.
 */

/*
class SilentCubitMessageHandler : public CubitMessageHandler
{

public:
  static int num_dropped_errors;

  SilentCubitMessageHandler(){}
  ~SilentCubitMessageHandler(){}

  virtual void print_message_prefix(const char *){}
  virtual void print_message(const char *){}
  virtual void print_error_prefix(const char *){}
  virtual void print_error(const char *){num_dropped_errors++;}
};

int SilentCubitMessageHandler::num_dropped_errors = 0;

/// RAII class to set up and shut down a SilentCubitMessageHandler 
class CubitSilence
{
protected:
  CubitMessageHandler* old_handler;
  SilentCubitMessageHandler silencer;
public:
  CubitSilence() :
    old_handler( CubitMessage::instance()->get_message_handler() )
  {
    CubitMessage::instance()->set_message_handler( &silencer );
  }

  ~CubitSilence(){
    CubitMessage::instance()->set_message_handler( old_handler );
  }
};
*/
#endif /* USING_CGMA */

typedef std::vector<iBase_EntityHandle> entity_collection_t;

/**
 * Contains geometry functions and the shared data members they all reference.
 */
class GeometryContext {

  /** 
   * Metadata and naming: 
   * The NamedGroup and NamedEntity mappings are used to keep track of metadata
   * on particular entity handles that map to MCNP cells.  
   * EntityHandles change frequently as CSG operations are performed on volumes,
   * so these mappings must be updated, by calling updateMaps(), whenever an 
   * EntityHandle changes.
   */

protected:

  // note: this appears slow, since it's called for all cells and constructs a string
  // for each.  A lookup table would probably be faster, but there are never more than
  // a few thousand cells.
  std::string materialName( int mat, double rho ){
    std::string ret;
    std::stringstream formatter;
    if(Gopt.uwuw_names){
      bool mass_density = false;
      if (rho <= 0){
        mass_density = true;
        rho = -rho;
      }
      char rho_formatted [50];
      sprintf(rho_formatted, "%E", rho);
      formatter << "mat:m" << mat;
      if(mass_density)
          formatter << "/rho:" <<rho_formatted;
      else
          formatter << "/atom:" << rho_formatted;
    }
    else
      formatter << "mat_" << mat << "_rho_" << rho;
    formatter >> ret;
    return ret;
  }

  std::string importanceName( char impchar, double imp ){
    std::string ret; 
    std::stringstream formatter;
    formatter << "imp." << impchar << "_" << imp;
    formatter >> ret;
    return ret;
  }

  class NamedGroup {
  protected:
    std::string name;
    entity_collection_t entities;
  public:
    NamedGroup( ) : name("") {}
    NamedGroup( std::string name_p ):
      name(name_p)
    {}

    const std::string& getName() const { return name; }
    const entity_collection_t& getEntities() const { return entities; }

    void add( iBase_EntityHandle new_handle ){
      entities.push_back(new_handle);
    }

    void update( iBase_EntityHandle old_h, iBase_EntityHandle new_h ){
      entity_collection_t::iterator i = std::find( entities.begin(), entities.end(), old_h );
      if( i != entities.end() ){
        if( new_h ){
          *i = new_h;
        }
        else {
          entities.erase( i );
        }
      }
    }

    bool contains( iBase_EntityHandle handle ) const {
      return std::find( entities.begin(), entities.end(), handle ) != entities.end(); 
    }

  };

  class NamedEntity { 
  protected:
    iBase_EntityHandle handle;
    std::string name;
  public:
    NamedEntity( iBase_EntityHandle handle_p, std::string name_p = "" ):
      handle(handle_p), name(name_p)
    {}
    virtual ~NamedEntity(){}

    const std::string& getName() const { return name; }
    iBase_EntityHandle getHandle() const{ return handle; }

    void setHandle( iBase_EntityHandle new_h ) {
      handle = new_h;
    }

    static NamedEntity* makeCellIDName( iBase_EntityHandle h, int ident ){
      NamedEntity* e = new NamedEntity(h);
      std::stringstream formatter;
      formatter << "MCNP_ID_" << ident;
      formatter >> e->name;
      return e;
    }

  };

protected:
  iGeom_Instance& igm;
  InputDeck& deck;
  double world_size;
  int universe_depth;

  std::map< std::string, NamedGroup* > named_groups;
  std::vector< NamedEntity* > named_cells;

  NamedGroup* getNamedGroup( const std::string& name ){
    if( named_groups.find( name ) == named_groups.end() ){
      named_groups[ name ] = new NamedGroup( name );
      if( OPT_DEBUG ) std::cout << "New named group: " << name 
                                << "num groups now " << named_groups.size() << std::endl;
    }
    return named_groups[ name ];
  }

private:

  GeometryContext(iGeom_Instance& igm_p, InputDeck& deck_p);
  ~GeometryContext() {};

  static GeometryContext* instance;

public:

  static GeometryContext* getInstance(iGeom_Instance& igm_p, InputDeck& deck_p);
  static GeometryContext* getInstance();

  bool defineLatticeNode( CellCard& cell, iBase_EntityHandle cell_shell, iBase_EntityHandle lattice_shell,
                          int x, int y, int z, entity_collection_t& accum );

  entity_collection_t defineCell( CellCard& cell, bool defineEmbedded, iBase_EntityHandle lattice_shell );
  entity_collection_t populateCell( CellCard& cell, iBase_EntityHandle cell_shell, iBase_EntityHandle lattice_shell );
 
  entity_collection_t defineUniverse( int universe, iBase_EntityHandle container, const Transform* transform );

  void addToVolumeGroup( iBase_EntityHandle cell, const std::string& groupname );
  void setVolumeCellID( iBase_EntityHandle cell, int ident);
  void setMaterial( iBase_EntityHandle cell, int material, double density ){
    if( Gopt.tag_materials ){
        addToVolumeGroup( cell, materialName(material,density) );
    }
  }

  void setImportances( iBase_EntityHandle cell, const std::map<char, double>& imps ){
    if( Gopt.tag_importances ){
      for( std::map<char, double>::const_iterator i = imps.begin();
           i != imps.end(); ++i ){
        char impchar = (*i).first;
        double imp = (*i).second;
        addToVolumeGroup( cell, importanceName( impchar, imp ) );
      }
    }
  }

  void updateMaps ( iBase_EntityHandle old_cell, iBase_EntityHandle new_cell );

  bool mapSanityCheck( iBase_EntityHandle* cells, size_t count );

  void tagGroups( );
  void tagCellIDsAsEntNames();

  std::string uprefix() {
    return std::string( universe_depth, ' ' );
  }

  iBase_EntityHandle createGraveyard( iBase_EntityHandle& boundary );
  void createGeometry( );

};

//#endif 
